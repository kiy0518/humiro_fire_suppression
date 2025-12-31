CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
INCLUDES = $(shell pkg-config --cflags opencv4) \
           $(shell pkg-config --cflags gstreamer-1.0) \
           $(shell pkg-config --cflags gstreamer-rtsp-server-1.0)
LIBS = $(shell pkg-config --libs opencv4) \
       $(shell pkg-config --libs gstreamer-1.0) \
       $(shell pkg-config --libs gstreamer-rtsp-server-1.0) \
       -lmicrohttpd -lpthread

SOURCES = main.cpp \
          utils.cpp \
          camera_manager.cpp \
          thermal_processor.cpp \
          frame_compositor.cpp \
          rtsp_server.cpp \
          http_server.cpp

OBJECTS = $(SOURCES:.cpp=.o)
TARGET = thermal_rgb_streaming

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $(TARGET) $(LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET)

install: $(TARGET)
	cp $(TARGET) /usr/local/bin/
