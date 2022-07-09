
GCC := gcc 
CXXFLAGS := -Wall 
LDFLAGS := ...

SRC_DIR_COMMON := src/common
SRC_DIR_6DOF := src/6DOF
SRC_DIR_1D := src/1D
SRC_DIR_2D := src/2D

SRC_FILES := $(wildcard $(SRC_DIR_COMMON)/src/*.cpp) $(wildcard $(SRC_DIR_6DOF)/src/*.cpp) $(wildcard $(SRC_DIR_1D)/src/*.cpp)
OBJ_FILES := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRC_FILES)) 

INCLUDE = -I/src/1D/include -I/src/6DOF/include -I/src/common/include
OBJDIR = obj


main.exe: $(OBJ_FILES)
    g++ $(LDFLAGS) -o $@ $^

$(OBJ_DIR)/%.o: $(SRC_DIR_COMMON)/src/%.cpp
    g++ $(CPPFLAGS) -c -o $@ $<

$(OBJ_DIR)/%.o: $(SRC_DIR_1D)/src/%.cpp
    g++ $(CPPFLAGS) -c -o $@ $<

$(OBJ_DIR)/%.o: $(SRC_DIR_2D)/src/%.cpp
    g++ $(CPPFLAGS) -c -o $@ $<

$(OBJ_DIR)/%.o: $(SRC_DIR_6DOF)/src/%.cpp
    g++ $(CPPFLAGS) -c -o $@ $<