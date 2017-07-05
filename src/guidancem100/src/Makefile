#define a compiler
CXX     = g++
#define target name
TARGET  = main
#define dependencies of target 
OBJECTS = main.o DJI_utility.o
#define the Include and Library path
CFLAGS  = -g -Wall -I/usr/local/include -I../../../include
LDFLAGS = -Wl,-rpath,./ -lpthread -lrt -L./ -L/usr/local/lib/ -lDJI_guidance -lusb-1.0 `pkg-config --cflags --libs opencv` 

$(TARGET) : $(OBJECTS)
	    $(CXX) -o $(TARGET) $(OBJECTS) $(LDFLAGS)
main.o : main.cpp DJI_utility.h
	    $(CXX) $(CFLAGS) -c main.cpp DJI_utility.h
DJI_utility.o : DJI_utility.cpp DJI_utility.h
	    $(CXX) $(CFLAGS) -c DJI_utility.cpp DJI_utility.h
clean:
	    rm -rf *.o *.gch *.avi $(TARGET)
