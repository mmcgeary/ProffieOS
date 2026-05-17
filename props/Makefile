test: style_registry_tests
	./style_registry_tests

style_registry_tests: style_registry_tests.cpp style_registry.h ini_loader.h runtime_config.h
	g++ -O -g -std=c++14 -I.. -MD -MP -o style_registry_tests style_registry_tests.cpp -lm

-include *.d
