CXX = g++

#Włącz te opcję, jeśli kompilujesz pod SFML 2.3.2 (domyślna w Ubuntu Xenial)
OLDSFML = -DOLD_VERSION
CXXFLAGS = -Wall -std=c++11 -O2 $(OLDSFML)
LIBS = -lsfml-graphics -lsfml-window -lsfml-system -lpthread
#Zakomentuj te opcje, jeśli używasz zainstalowanej wersji SFML
INCLUDEOPT = #-Iinclude
LIBOPT = #-Llib

SOURCES = $(shell ls *.cpp)
OBJECTS = $(subst /,/,$(SOURCES:.cpp=.o))
EXE = lalkarz

$(EXE): $(OBJECTS)
	$(CXX) -o $@ $(LIBOPT) $^ $(LIBS)
	
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $^ $(INCLUDEOPT)
	
.PHONY : clean
clean:
	-rm -f $(EXE) $(OBJECTS)
