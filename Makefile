BUILD_OUT := .build
WEB := web
NATIVE := native
BUILD_WEB := $(BUILD_OUT)/$(WEB)
BUILD_NATIVE := $(BUILD_OUT)/$(NATIVE)

# Helper
clean-web:
	rm -rf $(BUILD_WEB)

clean-native:
	rm -rf $(BUILD_NATIVE)

clean:
	rm -rf $(BUILD_OUT)

pre:
	mkdir -p $(BUILD_OUT)

# Browser
web: pre
	emcmake cmake -S . -B $(BUILD_WEB)
	emmake make -C $(BUILD_WEB)

serve: web
	python -m http.server -d $(BUILD_WEB)/bin/

# Desktop
native: pre
	cmake -S . -B $(BUILD_NATIVE)
	cmake --build $(BUILD_NATIVE)
	ln -sf $(BUILD_NATIVE)/compile_commands.json .

run: native
	$(BUILD_NATIVE)/bin/cpp_rlimgui_test
