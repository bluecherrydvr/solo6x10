CFLAGS		= -Wall
LIBS		= -lavcodec -lavformat -lavutil

TARGETS		= bc-record bc-disp-input

all: $(TARGETS)

bc-record: bc-record.c
	$(CC) $(CFLAGS) $< -o $@ $(LIBS)

bc-disp-input: bc-disp-input.c
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f $(TARGETS)
