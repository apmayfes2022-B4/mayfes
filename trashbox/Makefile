CC = g++
CFLAGS = -std=c++17
TARGET = bin/main
SRCS = main.cpp
OBJS = $(SRCS:.cpp=.o)
INCDIR = -I includes/include.hh
LIBDIR =
LIBS = 
$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LIDIR) $(LIBS) 
$(OBJS):$(SRCS)
	$(CC) $(CFLAGS) $(INCDIR) -c $(SRCS) 

all: clean $(OBJS) $(TARGET)
clean:
	-rm -f $(OBJS) $(TARGET) *.d
