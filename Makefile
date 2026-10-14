CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall
LIBS = -lsfml-graphics -lsfml-window -lsfml-system

TARGET = agv_sim
SRC = main.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(LIBS)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)

.PHONY: all run clean
