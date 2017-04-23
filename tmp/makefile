CXX = g++

CXX_FLAGS =-Wall -g --std=c++14
LIBS = -lsfml-graphics -lsfml-window -lsfml-system -lpthread
SOURCES = $(shell ls *.cpp)
OBJECTS = $(subst /,/,$(SOURCES:.cpp=.o))
EXE = lalkarz

$(EXE): $(OBJECTS)
	$(CXX) -o $@ $^ $(LDFLAGS) $(LIBS)
	
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $^
	
.PHONY : clean
clean:
	-rm -f $(EXE) $(OBJECTS)