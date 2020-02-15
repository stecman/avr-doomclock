SOURCES = test.c ../nmea.c

DEFS = -D__flash="" -D_GNU_SOURCE -DENABLE_GPS_DATE

test: $(SOURCES)
	gcc -std=c11 -Wall -I -g -o test $(SOURCES) $(DEFS)
	./test

clean:
	rm -f test