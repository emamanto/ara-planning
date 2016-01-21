PROJECT_DIR=$(PWD)
BIN_DIR=$(PROJECT_DIR)/bin
SRC_DIR=$(PROJECT_DIR)/src
GEN_MAKEFILE=$(BIN_DIR)/Makefile

MAKE_FLAGS=--no-print-directory

EXECUTABLE=arm_ara
MAZE_EXECUTABLE=maze_ara
ROBOT_EXECUTABLE=robot_ara
ROSIE_EXECUTABLE=rosie_ara
TEST_EXECUTABLE=tests
EXPERIMENT_EXECUTABLE=experiment

.phony: build build_maze build_robot build_rosie clean cfg run maze robot rosie test build_test exp build_exp

build_rosie: $(GEN_MAKEFILE)
	     make $(MAKE_FLAGS) -C $(BIN_DIR) $(ROSIE_EXECUTABLE)

build: $(GEN_MAKEFILE)
	make $(MAKE_FLAGS) -C $(BIN_DIR) $(EXECUTABLE)

build_maze: $(GEN_MAKEFILE)
	     make $(MAKE_FLAGS) -C $(BIN_DIR) $(MAZE_EXECUTABLE)

build_robot: $(GEN_MAKEFILE)
	     make $(MAKE_FLAGS) -C $(BIN_DIR) $(ROBOT_EXECUTABLE)

build_test: $(GEN_MAKEFILE)
	     make $(MAKE_FLAGS) -C $(BIN_DIR) $(TEST_EXECUTABLE)

build_exp: $(GEN_MAKEFILE)
	     make $(MAKE_FLAGS) -C $(BIN_DIR) $(EXPERIMENT_EXECUTABLE)

$(GEN_MAKEFILE):
	mkdir -p $(BIN_DIR); \
	cd $(BIN_DIR); \
	cmake $(SRC_DIR)

cfg: $(GEN_MAKEFILE)
	ccmake $(BIN_DIR)

clean:
	rm -rf $(BIN_DIR)

$(EXECUTABLE): build

$(MAZE_EXECUTABLE): build_maze

$(ROBOT_EXECUTABLE): build_robot

$(ROSIE_EXECUTABLE): build_rosie

$(TEST_EXECUTABLE): build_test

$(EXPERIMENT_EXECUTABLE): build_exp

run: $(EXECUTABLE)
	$(BIN_DIR)/$(EXECUTABLE)

maze: $(MAZE_EXECUTABLE)
	$(BIN_DIR)/$(MAZE_EXECUTABLE)

robot: $(ROBOT_EXECUTABLE)
	$(BIN_DIR)/$(ROBOT_EXECUTABLE)

rosie: $(ROSIE_EXECUTABLE)
	$(BIN_DIR)/$(ROSIE_EXECUTABLE)

test: $(TEST_EXECUTABLE)
	$(BIN_DIR)/$(TEST_EXECUTABLE)

exp: $(EXPERIMENT_EXECUTABLE)
	$(BIN_DIR)/$(EXPERIMENT_EXECUTABLE)
