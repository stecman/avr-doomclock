SOURCES = test.c ../nmea.c

DEFS = -D__flash="" -D_GNU_SOURCE -DENABLE_GPS_DATE

test: build
	./test

build: $(SOURCES)
	gcc -std=c11 -Wall -I -g -o test $(SOURCES) $(DEFS)

clean:
	rm -f test
