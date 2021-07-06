// SPDX-License-Identifier: MIT

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <limits.h>
#include <linux/dvb/frontend.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <unistd.h>

static uint32_t start_frequency_mhz = 950;
static uint32_t stop_frequency_mhz = 1950;
static uint32_t symbolrate_min_mhz = 2;
static uint32_t symbolrate_max_mhz = 45;
static bool vertical;
static bool cband;
static bool high;
static int slot;
static int i2c;

volatile sig_atomic_t signal_status;

static void signal_handler(int signal)
{
    signal_status = signal;
}

static void print_usage(char **argv)
{
    fprintf(stderr, "USAGE: %s [options]\n"
                    "\n"
                    "OPTIONS:\n"
                    "  -h, --help              Detailed usage\n"
                    "  -s, --start=<frequency> Scan start frequency in MHz\n"
                    "  -e, --stop=<frequency>  Scan stop frequency in MHz\n"
                    "  -n, --min=<symbolrate>  Minimum symbol rate to scan in MS/s\n"
                    "  -x, --max=<symbolrate>  Maximum symbol rate to scan in MS/s\n"
                    "  -V, --vertical          Signal polarity is vertical\n"
                    "  -C, --cband             Scan C-band\n"
                    "  -H, --high              Scan Ku-band high\n"
                    "  -S, --slot=<slot>       NIM slot (0...3)\n"
                    "  -I, --i2c=<id>          I2C device (0...3)\n",
            argv[0]);
}

static bool get_int_arg(int *valp, const char *arg)
{
    char *endptr;
    long int val;

    errno = 0;
    val = strtol(arg, &endptr, 10);

    if ((errno == ERANGE && (val == LONG_MAX || val == LONG_MIN)) || (errno != 0 && val == 0))
        return false;

    if (endptr == arg)
        return false;

    *valp = (int)val;
    return true;
}

static void handle_args(int argc, char **argv)
{
    struct option longopts[] = {
        {"start", required_argument, 0, 's'},
        {"stop", required_argument, 0, 'e'},
        {"min", required_argument, 0, 'n'},
        {"max", required_argument, 0, 'x'},
        {"vertical", no_argument, 0, 'V'},
        {"cband", no_argument, 0, 'C'},
        {"high", no_argument, 0, 'H'},
        {"slot", required_argument, 0, 'S'},
        {"i2c", required_argument, 0, 'I'},
        {"help", no_argument, 0, 'h'},
        {NULL, 0, 0, 0},
    };
    int c, longindex = 0, val;

    while ((c = getopt_long(argc, argv, "s:e:n:x:VCHS:I:h", longopts, &longindex)) != -1)
    {
        switch (c)
        {
        case 's':
            if (!get_int_arg(&val, optarg))
                exit(EXIT_FAILURE);
            start_frequency_mhz = val;
            break;
        case 'e':
            if (!get_int_arg(&val, optarg))
                exit(EXIT_FAILURE);
            stop_frequency_mhz = val;
            break;
        case 'n':
            if (!get_int_arg(&val, optarg))
                exit(EXIT_FAILURE);
            symbolrate_min_mhz = val;
            break;
        case 'x':
            if (!get_int_arg(&val, optarg))
                exit(EXIT_FAILURE);
            symbolrate_max_mhz = val;
            break;
        case 'V':
            vertical = true;
            break;
        case 'C':
            cband = true;
            break;
        case 'H':
            high = true;
            break;
        case 'S':
            if (!get_int_arg(&val, optarg))
                exit(EXIT_FAILURE);
            slot = val;
            break;
        case 'I':
            if (!get_int_arg(&val, optarg))
                exit(EXIT_FAILURE);
            i2c = val;
            break;
        case 'h':
        case '?':
            print_usage(argv);
            break;
        default:
            exit(EXIT_FAILURE);
            break;
        }
    }
}

static ssize_t bs_read(const char *filename, void *buf, size_t count)
{
    int fd;
    ssize_t rc = 0;
    ssize_t todo = count;

    fd = open(filename, O_RDWR);
    if (fd < 0)
        return -1;

    while (todo)
    {
        do
        {
            rc = read(fd, buf, todo);
        } while (rc < 0 && errno == EINTR);

        if (rc <= 0)
            break;

        buf = (char *)buf + rc;
        todo -= rc;
    }

    close(fd);

    return (count - todo) ? (count - todo) : rc;
}

static ssize_t bs_write(const char *filename, const void *buf, size_t count)
{
    int fd;
    ssize_t rc = 0;
    ssize_t todo = count;

    fd = open(filename, O_RDWR);
    if (fd < 0)
        return -1;

    while (todo)
    {
        do
        {
            rc = write(fd, buf, todo);
        } while (rc < 0 && errno == EINTR);

        if (rc <= 0)
            break;

        buf = (const char *)buf + rc;
        todo -= rc;
    }

    close(fd);

    return (count - todo) ? (count - todo) : rc;
}

static void blindscan(int fe_id)
{
    char bs_ctrl[PATH_MAX];
    char bs_info[PATH_MAX];
    char buf[BUFSIZ];
    int ret;
    int status, num_info, progress;
    int index;
    uint32_t frequency;
    uint32_t symbol_rate;
    int delivery_system;
    int inversion;
    int pilot;
    int fec_inner;
    int modulation;
    int rolloff;
    int pls_mode;
    int is_id;
    int pls_code;
    int t2mi_plp_id;
    int t2mi_pid;
    char *s;

    sprintf(bs_ctrl, "/proc/stb/frontend/%d/bs_ctrl", fe_id);
    if (access(bs_ctrl, R_OK))
        return;

    sprintf(bs_info, "/proc/stb/frontend/%d/bs_info", fe_id);
    if (access(bs_info, R_OK))
        return;

    sprintf(buf, "1 %u %u %u %u",
            start_frequency_mhz, stop_frequency_mhz,
            symbolrate_min_mhz, symbolrate_max_mhz);
    ret = bs_write(bs_ctrl, buf, strlen(buf));
    if (ret < 0)
        return;

    for (;;)
    {
        if (signal_status == SIGINT)
        {
            sprintf(buf, "0 0 0 0 0");
            bs_write(bs_ctrl, buf, strlen(buf));
            return;
        }

        ret = bs_read(bs_ctrl, buf, sizeof(buf) - 1);
        if (ret < 0)
            return;

        buf[ret] = '\0';
        sscanf(buf, "%d%d%d", &status, &num_info, &progress);
        if (!status)
            break;

        usleep(100 * 1000);
    }

    for (int i = 0; i < num_info && signal_status != SIGINT; i++)
    {
        sprintf(buf, "%d", i);
        ret = bs_write(bs_info, buf, strlen(buf));
        if (ret < 0)
            continue;

        ret = bs_read(bs_info, buf, sizeof(buf) - 1);
        if (ret < 0)
            continue;

        buf[ret] = '\0';
        if (sscanf(buf, "%d%u%u%d%d%d%d%d%d%d%d%d%d%d",
                   &index,
                   &frequency,
                   &symbol_rate,
                   &delivery_system,
                   &inversion,
                   &pilot,
                   &fec_inner,
                   &modulation,
                   &rolloff,
                   &pls_mode,
                   &is_id,
                   &pls_code,
                   &t2mi_plp_id,
                   &t2mi_pid) != 14)
            continue;

        if (i != index)
            continue;

        int j = sprintf(buf, "OK");

        j += sprintf(buf + j, " %s", vertical ? "VERTICAL" : "HORIZONTAL");

        frequency = ((frequency + 500) / 1000) * 1000;
        if (cband)
            frequency = 5150000U - frequency;
        else if (high)
            frequency = frequency + 10600000U;
        else
            frequency = frequency + 9750000U;

        j += sprintf(buf + j, " %u", frequency);

        symbol_rate = ((symbol_rate + 500) / 1000) * 1000;

        j += sprintf(buf + j, " %u", symbol_rate);

        j += sprintf(buf + j, " %s", delivery_system == SYS_DVBS ? "DVB-S" : "DVB-S2");

        switch (inversion)
        {
        case INVERSION_OFF:
            s = "INVERSION_OFF";
            break;
        case INVERSION_ON:
            s = "INVERSION_ON";
            break;
        default:
            s = "INVERSION_AUTO";
            break;
        }

        j += sprintf(buf + j, " %s", s);

        switch (pilot)
        {
        case PILOT_ON:
            s = "PILOT_ON";
            break;
        case PILOT_OFF:
            s = "PILOT_OFF";
            break;
        default:
            s = "PILOT_AUTO";
            break;
        }

        j += sprintf(buf + j, " %s", s);

        switch (fec_inner)
        {
        case FEC_1_2:
            s = "FEC_1_2";
            break;
        case FEC_2_3:
            s = "FEC_2_3";
            break;
        case FEC_3_4:
            s = "FEC_3_4";
            break;
        case FEC_4_5:
            s = "FEC_4_5";
            break;
        case FEC_5_6:
            s = "FEC_5_6";
            break;
        case FEC_6_7:
            s = "FEC_6_7";
            break;
        case FEC_7_8:
            s = "FEC_7_8";
            break;
        case FEC_8_9:
            s = "FEC_8_9";
            break;
        case FEC_3_5:
            s = "FEC_3_5";
            break;
        case FEC_9_10:
            s = "FEC_9_10";
            break;
        case FEC_2_5:
            s = "FEC_2_5";
            break;
        default:
            s = "FEC_AUTO";
            break;
        }

        j += sprintf(buf + j, " %s", s);

        switch (modulation)
        {
        case PSK_8:
            s = "8PSK";
            break;
        case APSK_16:
            s = "16APSK";
            break;
        case APSK_32:
            s = "32APSK";
            break;
        default:
            s = "QPSK";
            break;
        }

        j += sprintf(buf + j, " %s", s);

        switch (rolloff)
        {
        case ROLLOFF_20:
            s = "ROLLOFF_20";
            break;
        case ROLLOFF_25:
            s = "ROLLOFF_25";
            break;
        default:
            s = "ROLLOFF_35";
            break;
        }

        j += sprintf(buf + j, " %s", s);

        j += sprintf(buf + j, " %d", pls_mode);

        j += sprintf(buf + j, " %d", is_id);

        j += sprintf(buf + j, " %d", pls_code);

        if (t2mi_plp_id != -1)
        {
            j += sprintf(buf + j, " %d", t2mi_plp_id);

            j += sprintf(buf + j, " %d", t2mi_pid);
        }

        j += sprintf(buf + j, "\n");

        fprintf(stdout, "%s", buf);
        fflush(stdout);
    }
}

static int nim_sockets(void)
{
    FILE *fp;
    char *line = NULL;
    size_t len = 0;
    ssize_t n;
    int ids[4] = {-1, -1, -1, -1};
    int *id;
    int fe_id = -1;

    fp = fopen("/proc/bus/nim_sockets", "r");
    if (fp == NULL)
        return fe_id;

    while ((n = getline(&line, &len, fp)) != -1)
    {
        int val;

        if (strstr(line, "NIM Socket") == line)
        {
            sscanf(line, "NIM Socket %d", &val);
            id = &ids[val];
        }
        else if (strstr(line, "\tFrontend_Device") == line)
        {
            sscanf(line, "\tFrontend_Device: %d", &val);
            *id = val;
        }
    }

    fclose(fp);
    if (line)
        free(line);

    for (int i = 0; i < 4; i++)
    {
        if (ids[i] == slot)
        {
            fe_id = ids[i];
            break;
        }
    }

    return fe_id;
}

int main(int argc, char **argv)
{
    int fd;
    char str[10];
    int fe_id;

    handle_args(argc, argv);

    fd = open("/var/run/blindscan.pid", O_RDWR | O_CREAT, 0664);
    if (fd < 0)
        exit(EXIT_FAILURE);

    if (flock(fd, LOCK_EX | LOCK_NB))
        exit(EXIT_FAILURE);

    sprintf(str, "%d\n", getpid());
    if (write(fd, str, strlen(str)) == -1)
        exit(EXIT_FAILURE);

    signal(SIGINT, signal_handler);

    sleep(5);

    fe_id = nim_sockets();
    if (fe_id != -1)
        blindscan(fe_id);

    flock(fd, LOCK_UN);
    close(fd);

    return 0;
}
