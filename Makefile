PLUGIN_NAME := LosTT
BUNDLE := $(PLUGIN_NAME).lv2
TARGET := $(BUNDLE)/$(PLUGIN_NAME).so
OBJ := $(BUNDLE)/lostt.o
SRC := src/lostt.cpp

CXX ?= g++
CC ?= cc
CXXFLAGS ?= -O3 -fPIC -std=c++17 -Wall -Wextra -fno-exceptions -fno-rtti -fvisibility=hidden -Iinclude
LDFLAGS ?= -shared -Wl,--as-needed
LDLIBS ?= -lm

.RECIPEPREFIX := >

.PHONY: all clean check install

all: $(TARGET) $(BUNDLE)/manifest.ttl $(BUNDLE)/$(PLUGIN_NAME).ttl

$(BUNDLE):
>mkdir -p $(BUNDLE)

$(OBJ): $(SRC) | $(BUNDLE)
>$(CXX) $(CXXFLAGS) -c $< -o $@

$(TARGET): $(OBJ)
>$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(BUNDLE)/manifest.ttl: manifest.ttl | $(BUNDLE)
>cp $< $@

$(BUNDLE)/$(PLUGIN_NAME).ttl: $(PLUGIN_NAME).ttl | $(BUNDLE)
>cp $< $@

clean:
>rm -rf $(BUNDLE)

check: all
>test -f $(TARGET)
>test -f $(BUNDLE)/manifest.ttl
>test -f $(BUNDLE)/$(PLUGIN_NAME).ttl
>nm -D $(TARGET) | grep -q 'lv2_descriptor'
>echo 'check: ok'

install: all
>mkdir -p $$HOME/.lv2/$(BUNDLE)
>cp -f $(TARGET) $(BUNDLE)/manifest.ttl $(BUNDLE)/$(PLUGIN_NAME).ttl $$HOME/.lv2/$(BUNDLE)/
