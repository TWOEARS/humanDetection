lib_LTLIBRARIES = libhumandetection_codels.la

libhumandetection_codels_la_SOURCES=	humandetection_c_types.h
libhumandetection_codels_la_SOURCES+=	humandetection_codels.cc
libhumandetection_codels_la_SOURCES+=	humandetection_Detection_codels.cc

#libhumandetection_codels_la_SOURCES+=	Misc.h
#libhumandetection_codels_la_SOURCES+=	Misc.cpp

libhumandetection_codels_la_SOURCES+=	triangulation.h
libhumandetection_codels_la_SOURCES+=	triangulation.cpp

libhumandetection_codels_la_SOURCES+=	Ports.h
libhumandetection_codels_la_SOURCES+=	Ports.cpp

LAPACK_LIBDIR=/usr/lib/lapack

libhumandetection_codels_la_CPPFLAGS=	$(requires_CFLAGS)
libhumandetection_codels_la_LDFLAGS=	$(requires_LIBS)
libhumandetection_codels_la_LDFLAGS+=	-release $(PACKAGE_VERSION)
libhumandetection_codels_la_LDFLAGS+=	-L$(LAPACK_LIBDIR) -llapack

# idl  mappings
BUILT_SOURCES=	humandetection_c_types.h
CLEANFILES=	${BUILT_SOURCES}
humandetection_c_types.h: ${top_srcdir}/humandetection.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/humandetection_c_types.h.d
