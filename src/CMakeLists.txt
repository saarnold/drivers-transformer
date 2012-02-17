rock_library(transformer
    SOURCES Transformer.cpp
	    NonAligningTransformer.cpp
    HEADERS Transformer.hpp TransformationStatus.hpp
	    NonAligningTransformer.hpp
    DEPS_PKGCONFIG aggregator base-types)

