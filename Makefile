# SPDX-License-Identifier: MIT

all: blindscan

blindscan: blindscan.o
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	-rm -f *.o blindscan
