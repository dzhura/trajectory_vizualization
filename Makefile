CXX=g++ -std=c++11

CXXFLAGS= -Wall -Wextra -O2 `pkg-config --cflags opencv` # compilation flags
#CXXFLAGS= -Wall -Wextra -ggdb `pkg-config --cflags opencv` 
LDFLAGS= `pkg-config --libs opencv` #flags for ld

EXECUTABLE= trajectory_vizualization

SOURCES= main.cpp
OBJECTS= $(SOURCES:.cpp=.o)

#all: $(SOURCES) $(EXECUTABLE)
$(EXECUTABLE): $(OBJECTS)
	$(CXX) $(LDFLAGS) $(OBJECTS) -o $@

main.o: main.cpp

.PHONY: clean
clean:
# '-rm' - ignore errors
	-rm $(OBJECTS) $(EXECUTABLE)

# $@ - filename of the target of the rule
# $< - the name of first prerequisite
# $? - names of all prerequisites, that are newer than target
# $^ - names of all prerequisites
