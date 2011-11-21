CFLAGS		= -Wall
LIBS		= -lavcodec -lavformat -lavutil

TARGET		= bc-record

all: $(TARGET)

$(TARGET): $(TARGET).c
	$(CC) $(CFLAGS) $< -o $@ $(LIBS)

clean:
	rm -f $(TARGET)
