AM_CFLAGS   = -Wall -Wextra -Wno-sign-compare -Wno-unused-parameter
AM_CPPFLAGS = -DCONFIG_BPG_VERSION=\"@VERSION@\"
AM_CXXFLAGS = $(AM_CFLAGS)



bin_PROGRAMS    = bpgdec bpgenc bpgmux bpgview
lib_LTLIBRARIES = libbpg.la
include_HEADERS = libbpg.h

pkgconfiglibdir   = $(libdir)/pkgconfig
pkgconfiglib_DATA = libbpg.pc

doc_DATA  = ChangeLog README doc/bpg_spec.txt
man1_MANS = man/bpgdec.1 man/bpgenc.1 man/bpgview.1
man5_MANS = man/bpg.5

CLEANFILES = $(man1_MANS) $(man5_MANS)



libbpg_la_SOURCES = \
		libavcodec/hevc_cabac.c \
		libavcodec/hevc_filter.c \
		libavcodec/hevc.c \
		libavcodec/hevcpred.c \
		libavcodec/hevc_refs.c \
		libavcodec/hevcdsp.c \
		libavcodec/hevc_mvs.c \
		libavcodec/hevc_ps.c \
		libavcodec/hevc_sei.c \
		libavcodec/utils.c \
		libavcodec/cabac.c \
		libavcodec/golomb.c \
		libavcodec/videodsp.c \
		libavutil/mem.c \
		libavutil/buffer.c \
		libavutil/log2_tab.c \
		libavutil/frame.c \
		libavutil/pixdesc.c \
		libavutil/md5.c \
		libbpg.c
libbpg_la_CFLAGS   = $(AM_CFLAGS) -Ilibavcodec
libbpg_la_CPPFLAGS = $(AM_CPPFLAGS) \
		-D_ISOC99_SOURCE -D_POSIX_C_SOURCE=200112 -D_XOPEN_SOURCE=600 \
		-DHAVE_AV_CONFIG_H -std=c99 -D_GNU_SOURCE=1 -DUSE_VAR_BIT_DEPTH -DUSE_PRED

# name the shared library libbpg-VERSION.so.0 until the first official
# libbpg.so.0 version is released
libbpg_la_LDFLAGS = -release $(VERSION) -version-info 0:0:0



bpgdec_SOURCES = bpgdec.c
bpgdec_LDADD   = libbpg.la $(PNG16_LIBS)



bpgenc_SOURCES  = bpgenc.c bpgfmt.c
bpgenc_CFLAGS   = $(AM_CFLAGS) $(PNG16_CFLAGS)
bpgenc_CPPFLAGS = $(AM_CPPFLAGS)
bpgenc_CXXFLAGS = $(AM_CXXFLAGS)
bpgenc_LDADD    = libbpg.la $(PNG16_LIBS)

if USE_JCTVC
bpgenc_SOURCES += jctvc_glue.cpp \
		jctvc/TLibVideoIO/TVideoIOYuv.cpp \
		jctvc/program_options_lite.cpp \
		jctvc/TLibEncoder/SyntaxElementWriter.cpp \
		jctvc/TLibEncoder/TEncSearch.cpp \
		jctvc/TLibEncoder/TEncGOP.cpp \
		jctvc/TLibEncoder/TEncTop.cpp \
		jctvc/TLibEncoder/TEncBinCoderCABACCounter.cpp \
		jctvc/TLibEncoder/TEncCavlc.cpp \
		jctvc/TLibEncoder/TEncCu.cpp \
		jctvc/TLibEncoder/TEncSlice.cpp \
		jctvc/TLibEncoder/TEncPic.cpp \
		jctvc/TLibEncoder/TEncSampleAdaptiveOffset.cpp \
		jctvc/TLibEncoder/WeightPredAnalysis.cpp \
		jctvc/TLibEncoder/TEncBinCoderCABAC.cpp \
		jctvc/TLibEncoder/SEIwrite.cpp \
		jctvc/TLibEncoder/TEncRateCtrl.cpp \
		jctvc/TLibEncoder/TEncPreanalyzer.cpp \
		jctvc/TLibEncoder/NALwrite.cpp \
		jctvc/TLibEncoder/TEncAnalyze.cpp \
		jctvc/TLibEncoder/TEncSbac.cpp \
		jctvc/TLibEncoder/TEncEntropy.cpp \
		jctvc/TLibCommon/TComWeightPrediction.cpp \
		jctvc/TLibCommon/TComPattern.cpp \
		jctvc/TLibCommon/TComSampleAdaptiveOffset.cpp \
		jctvc/TLibCommon/TComPicSym.cpp \
		jctvc/TLibCommon/TComPicYuvMD5.cpp \
		jctvc/TLibCommon/SEI.cpp \
		jctvc/TLibCommon/ContextModel3DBuffer.cpp \
		jctvc/TLibCommon/Debug.cpp \
		jctvc/TLibCommon/TComPicYuv.cpp \
		jctvc/TLibCommon/TComInterpolationFilter.cpp \
		jctvc/TLibCommon/TComTrQuant.cpp \
		jctvc/TLibCommon/TComRdCostWeightPrediction.cpp \
		jctvc/TLibCommon/TComLoopFilter.cpp \
		jctvc/TLibCommon/TComChromaFormat.cpp \
		jctvc/TLibCommon/TComSlice.cpp \
		jctvc/TLibCommon/TComDataCU.cpp \
		jctvc/TLibCommon/TComTU.cpp \
		jctvc/TLibCommon/TComCABACTables.cpp \
		jctvc/TLibCommon/TComPic.cpp \
		jctvc/TLibCommon/TComRom.cpp \
		jctvc/TLibCommon/TComYuv.cpp \
		jctvc/TLibCommon/TComPrediction.cpp \
		jctvc/TLibCommon/TComMotionInfo.cpp \
		jctvc/TLibCommon/TComBitStream.cpp \
		jctvc/TLibCommon/ContextModel.cpp \
		jctvc/TLibCommon/TComRdCost.cpp \
		jctvc/TAppEncTop.cpp \
		jctvc/TAppEncCfg.cpp \
		jctvc/libmd5/libmd5.c
bpgenc_CPPFLAGS += -DUSE_JCTVC
bpgenc_CXXFLAGS += -Ijctvc
endif

if USE_X265
bpgenc_SOURCES  += x265_glue.c
bpgenc_CPPFLAGS += -DUSE_X265
bpgenc_LDADD    += -lx265
endif



bpgmux_SOURCES = bpgmux.c bpgfmt.c



bpgview_SOURCES = bpgview.c
bpgview_LDADD   = libbpg.la $(SDL_LIBS) $(SDL_IMAGE_LIBS)



# hack libbpg.la to install the library as libbpg-VERSION.so.0 and create the following
# symbolic links to it: libbpg-VERSION.so libbpg.so
all-local:
	[ ! -f libbpg.la ] || sed -i "s/library_names='libbpg-$(VERSION).so.0.0.0 libbpg-$(VERSION).so.0 libbpg.so'/library_names='libbpg-$(VERSION).so.0 libbpg-$(VERSION).so libbpg.so'/" libbpg.la



make_man1 = cat man/templates/$^.1.header > $@; ./$^ >> $@ || true; cat man/templates/$^.1.footer >> $@

man/bpgdec.1: bpgdec
	$(make_man1)

man/bpgenc.1: bpgenc
	$(make_man1)

man/bpgmux.1: bpgmux
	$(make_man1)

man/bpgview.1: bpgview
	$(make_man1)

man/bpg.5:
	cat man/templates/bpg.5.header > $@
	cat doc/bpg_spec.txt >> $@
	cat man/templates/bpg.5.footer >> $@



# remove '-ljpeg' from the dependency line inside libbpg.la since
# the library doesn't actually depend on it (temporary solution)
install-exec-hook:
	[ ! -f $(DESTDIR)$(libdir)/libbpg.la ] || sed -i 's/-ljpeg//' $(DESTDIR)$(libdir)/libbpg.la


