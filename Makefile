PROJECT_DIR=$(PWD)
BIN_DIR=$(PROJECT_DIR)/bin
SRC_DIR=$(PROJECT_DIR)/src
GEN_MAKEFILE=$(BIN_DIR)/Makefile

MAKE_FLAGS=--no-print-directory

EXECUTABLE=$(BIN_DIR)/ara

.phony: build clean cfg

build: $(GEN_MAKEFILE)
	make $(MAKE_FLAGS) -C $(BIN_DIR)

$(GEN_MAKEFILE):
	mkdir -p $(BIN_DIR); \
	cd $(BIN_DIR); \
	cmake $(SRC_DIR)

cfg: $(GEN_MAKEFILE)
	ccmake $(BIN_DIR)

clean:
	rm -rf $(BIN_DIR)

$(EXECUTABLE): build

run: $(EXECUTABLE)
	$(EXECUTABLE)
