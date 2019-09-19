# Usage:
# make        # compile all binary
# make clean  # remove ALL binaries and objects

CPPFLAGS = -c -Os -std=c++11 -Weffc++ -Wall -Wextra -Wshadow -Wnon-virtual-dtor -pedantic \
        -I include -D SINGLETIME_COMMENTS_ENABLED -D DEBUGGING_INFO_ENABLED
LFLAGS = -Wall -Wextra -Os
LIBS = -lpthread
SRC_DIR = .
OBJ_DIR = .
TARGET_DIR = .        
CXX = arm-linux-gnueabihf-g++                       # compiler to use
#CC = arm-linux-gnueabihf-g++                       # compiler to use
#CXX=g++

#LINKERFLAG = -lm

SRC := $(wildcard $(SRC_DIR)/*.cpp) # src/blue.cpp ...
OBJ := $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SRC)) # obj/blue.o ...
TARGET = ./multiUARTApp

.PHONY: clean

$(TARGET): $(OBJ)
	$(CXX) $(LFLAGS) -o $@  $^ $(LIBS)
	@echo Build successful

$(OBJ): $(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CPPFLAGS) -o $@ $< 

clean:
	rm -rf *.o build/* bin/* *~
