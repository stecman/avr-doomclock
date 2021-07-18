SOURCES = test.c ../nmea.c

DEFS = -D__flash="" # Remove keyword as it's not valid for regular GCC
DEFS += -DAVRSTATIC="" # Don't add static keyword to header functions
DEFS += -DENABLE_GPS_DATE # Test date the optional date parsing
DEFS += -D_GNU_SOURCE # Allow use of asprintf

test: build
	./test

build: $(SOURCES)
	gcc -std=c11 -Wall -I -g -o test $(SOURCES) $(DEFS)

clean:
	rm -f test
