all:	Gen/float/testbed Gen/fixed/testbed

clean:
	rm -rf Gen

PROJECT=	../..

CXXFLAGS=	-g -O2 -I$(PROJECT)/Contrib -I$(PROJECT)/Include -DGLUI_FREEGLUT

LDFLAGS=	-L$(PROJECT)/Contrib/freeglut -L$(PROJECT)/Contrib/glui \

SOURCES=	Framework/Main.cpp \
		Framework/Test.cpp \
		Framework/Render.cpp \
		Tests/BroadPhaseTest.cpp \
		Tests/Biped.cpp \
		Tests/BipedDef.cpp \
		Tests/TestEntries.cpp 

FLOAT_OBJECTS= $(addprefix Gen/float/,$(SOURCES:.cpp=.o))

Gen/float/%.o:		%.cpp
	mkdir -p $(dir $@)
	c++ $(CXXFLAGS) -c -o $@ $<

Gen/float/testbed:	$(FLOAT_OBJECTS)
	g++ -o $@ $^ $(LDFLAGS) -L$(PROJECT)/Source/Gen/float -lbox2d -lglui -lglut -lGLU -lGL


FIXED_OBJECTS= $(addprefix Gen/fixed/,$(SOURCES:.cpp=.o))

Gen/fixed/%.o:		%.cpp
	mkdir -p $(dir $@)
	c++ $(CXXFLAGS) -DTARGET_FLOAT32_IS_FIXED -c -o $@ $<

Gen/fixed/testbed:	$(FIXED_OBJECTS)
	g++ -o $@ $^ $(LDFLAGS) -L$(PROJECT)/Source/Gen/fixed -lbox2d -lglui -lglut -lGLU -lGL
