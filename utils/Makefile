CFLAGS		= -Wall
LIBS		= -lavcodec -lavformat -lavutil

TARGETS		= bc-record bluecherry-display

all: $(TARGETS)

bc-record: bc-record.c
	$(CC) $(CFLAGS) $< -o $@ $(LIBS)

bluecherry-display: bluecherry-display.c
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f $(TARGETS)
