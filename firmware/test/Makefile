SOURCES = test.c ../nmea.c

test: $(SOURCES)
	gcc -std=c11 -Wall -I -g -o test $(SOURCES) -D__flash="" -D_GNU_SOURCE
	./test

clean:
	rm -f test