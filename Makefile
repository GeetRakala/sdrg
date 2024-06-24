# Compiler
CXX = g++

# Boost include and library paths
BOOST_INCLUDE = $(wildcard /opt/homebrew/Cellar/boost/*/include)
BOOST_LIB = $(wildcard /opt/homebrew/Cellar/boost/*/lib)

# Compiler flags
#CXXFLAGS = -std=c++17 -Iinclude -I$(BOOST_INCLUDE)
CXXFLAGS = -o3 -std=c++17 -I$(BOOST_INCLUDE)

# Linker flags
LDFLAGS = -L$(BOOST_LIB)

# Debug flags
DEBUG_FLAGS = -fsanitize=address -g

# Source files
SOURCES = $(wildcard src/*.cpp)

# Object files
OBJECTS = $(addprefix build/, $(notdir $(SOURCES:.cpp=.o)))

# Executable name
EXEC = do_sdrg

# Default target
all: $(EXEC)
	@echo
	@echo "Program has successfully compiled. Run 'make run' to run the program."
	@echo

$(EXEC): $(OBJECTS)
	@$(CXX) $(LDFLAGS) $^ -o $@ || (echo "Compilation failed. Check the error message above."; echo "For more details, try running with 'make debug'."; exit 1)

debug: CXXFLAGS += $(DEBUG_FLAGS)
debug: LDFLAGS += $(DEBUG_FLAGS)
debug: $(EXEC)
	@echo
	@echo "Program has successfully compiled in debug mode. Run 'lldb ./$(EXEC)' to run the program in debug mode."
	@echo

# Pattern rule for object files
build/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean the output files and run the program
run:
	rm -f json/*
	rm -f pdf/*
	@echo
	@echo "Cleaned the json/ and pdf/ directories."
	@echo
	time ./$(EXEC)

# Clean target
clean:
	rm -f $(OBJECTS) $(EXEC)
	rm -f json/*
	rm -f pdf/*
	rm -f csvfiles/*
	@echo
	@echo "Removed build files and executable."
	@echo "Cleaned the json/, pdf/ and csvfiles/ directories."
	@echo

# Clean only the output files
clean-output:
	rm -f json/*
	rm -f pdf/*
	rm -f csvfiles/*
	@echo
	@echo "Cleaned the json/, pdf/ and csvfiles/ directories."
	@echo

.PHONY: all debug clean
