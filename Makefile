include ffmpeg/config.mak

# CXXFLAGS += -Wconversion -Wno-sign-conversion
# -O2
CXX := clang++
CC := clang
CXXFLAGS += -O3 -std=c++1y -Wall -g -I. -I./ffmpeg \
    $(shell pkg-config --cflags protobuf) \
		-pthread
LDLIBS = -L./ffmpeg/libavdevice -lavdevice \
	 -L./ffmpeg/libavformat -lavformat \
	 -L./ffmpeg/libavfilter -lavfilter \
	 -L./ffmpeg/libavcodec -lavcodec \
	 -L./ffmpeg/libswresample -lswresample \
	 -L./ffmpeg/libswscale -lswscale \
	 -L./ffmpeg/libavutil -lavutil \
	 $(shell pkg-config --libs protobuf) \
	 -pthread -lpthread \
	 $(EXTRALIBS) \
	 -lstdc++

#CXXFLAGS += -std=c++1y -Wall -g -I. -I./ffmpeg \
	   $(shell pkg-config --cflags protobuf)
#LDLIBS = -L./ffmpeg/libavdevice -lavdevice \
	 -L./ffmpeg/libavformat -lavformat \
	 -L./ffmpeg/libavfilter -lavfilter \
	 -L./ffmpeg/libavcodec -lavcodec \
	 -L./ffmpeg/libswresample -lswresample \
	 -L./ffmpeg/libswscale -lswscale \
	 -L./ffmpeg/libavutil -lavutil \
	 $(EXTRALIBS) \
	 $(shell pkg-config --libs protobuf) \
	 -lstdc++

recode: recode.o recode.pb.o ffmpeg/libavcodec/libavcodec.a

recode.o: recode.cpp recode.pb.h arithmetic_code.h cabac_code.h \
    recode.h av_decoder.h neighbors.h h264_model.h h264_symbol.h compressor.h \
    decompressor.h nd_array.h

recode.pb.cc recode.pb.h: recode.proto
	protoc --cpp_out=. $<


test/arithmetic_code: test/arithmetic_code.o

test/arithmetic_code.o: test/arithmetic_code.cpp arithmetic_code.h cabac_code.h

clean:
	rm -f recode recode.o recode.pb.{cc,h,o}
