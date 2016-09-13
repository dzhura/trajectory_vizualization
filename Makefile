CC=gcc
CXX=g++ -std=c++11

CFLAGS=-O2

CXXFLAGS= -Wall -Wextra -O2 -I. `pkg-config --cflags opencv`
#CXXFLAGS= -Wall -Wextra -ggdb `pkg-config --cflags opencv` 
LDFLAGS= `pkg-config --libs opencv`

EXECUTABLE= trajectory_vizualization

SOURCES= filters.cpp gnuplot_i.c main.cpp
OBJECTS= $(addsuffix .o,$(basename $(SOURCES)))

#all: $(SOURCES) $(EXECUTABLE)
$(EXECUTABLE): $(OBJECTS)
	$(CXX) $(LDFLAGS) $(OBJECTS) -o $@

main.o: filters.hpp gnuplot_i.h
filters.o: filters.hpp
gnuplot_i.o: gnuplot_i.h

.PHONY: clean
clean:
# '-rm' - ignore errors
	-rm $(OBJECTS) $(EXECUTABLE)

# $@ - filename of the target of the rule
# $< - the name of first prerequisite
# $? - names of all prerequisites, that are newer than target
# $^ - names of all prerequisites
