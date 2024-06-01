# specify the compiler
CC=g++

INCLUDE=-Iinclude -I"../include/lib15442c"

# specify options for the compiler
CFLAGS=-c -Wall $(INCLUDE)

# specify the target directory for the executable
BINDIR=bin

# specify the target directory for object files
OBJDIR=obj

# use wildcard to find all .cpp files in src directory
SRCS=$(wildcard src/*.cpp) $(wildcard ../src/lib15442c/*.cpp)

# create a list of object files by replacing .cpp with .o,
# and prefixing with obj/
OBJS=$(patsubst src/%.cpp,$(OBJDIR)/%.o,$(SRCS)) $(patsubst ../src/lib15442c/%.cpp,$(OBJDIR)/lib15442c/%.o,$(SRCS))

# specify the external libraries
LIBS=

# default target
all: $(BINDIR)/test

$(BINDIR)/test: $(OBJS)
	$(CC) $^ -o $@ $(LIBS)

$(OBJDIR)/lib15442c/%.o: ../src/lib15442c/%.cpp
	$(CC) $(CFLAGS) $< -o $@

$(OBJDIR)/%.o: src/%.cpp
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm $(OBJDIR)/*.o $(BINDIR)/hello
