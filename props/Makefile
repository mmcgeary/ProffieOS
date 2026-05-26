SCHEMA_GENERATOR := tools/generate_style_schema.py
SCHEMA_SOURCE := style_schema.json
GENERATED_SCHEMA_HEADER := generated_style_schema.h

test: style_registry_tests
	./style_registry_tests

style_registry_tests: style_registry_tests.cpp style_registry.h ini_loader.h runtime_config.h $(GENERATED_SCHEMA_HEADER)
	g++ -O -g -std=c++14 -I.. -MD -MP -o style_registry_tests style_registry_tests.cpp -lm

$(GENERATED_SCHEMA_HEADER): $(SCHEMA_SOURCE) $(SCHEMA_GENERATOR)
	python3 $(SCHEMA_GENERATOR)

-include *.d
