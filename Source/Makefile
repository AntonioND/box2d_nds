TARGETS= Gen/float/libbox2d.a Gen/fixed/libbox2d.a

ifdef DEVKITPRO
TARGETS += Gen/nds-float/lib/libbox2d.a Gen/nds-fixed/lib/libbox2d.a
endif

all:	$(TARGETS)

clean:
	rm -rf Gen

CXXFLAGS=	-g -O2

SOURCES = \
	./Dynamics/b2Body.cpp \
	./Dynamics/b2Island.cpp \
	./Dynamics/b2World.cpp \
	./Dynamics/b2ContactManager.cpp \
	./Dynamics/Contacts/b2Contact.cpp \
	./Dynamics/Contacts/b2PolyContact.cpp \
	./Dynamics/Contacts/b2CircleContact.cpp \
	./Dynamics/Contacts/b2PolyAndCircleContact.cpp \
	./Dynamics/Contacts/b2ContactSolver.cpp \
	./Dynamics/b2WorldCallbacks.cpp \
	./Dynamics/Joints/b2MouseJoint.cpp \
	./Dynamics/Joints/b2PulleyJoint.cpp \
	./Dynamics/Joints/b2Joint.cpp \
	./Dynamics/Joints/b2RevoluteJoint.cpp \
	./Dynamics/Joints/b2PrismaticJoint.cpp \
	./Dynamics/Joints/b2DistanceJoint.cpp \
	./Dynamics/Joints/b2GearJoint.cpp \
	./Common/b2StackAllocator.cpp \
	./Common/b2Math.cpp \
	./Common/b2BlockAllocator.cpp \
	./Common/b2Settings.cpp \
	./Collision/b2Collision.cpp \
	./Collision/b2Distance.cpp \
	./Collision/Shapes/b2Shape.cpp \
	./Collision/Shapes/b2CircleShape.cpp \
	./Collision/Shapes/b2PolygonShape.cpp \
	./Collision/b2TimeOfImpact.cpp \
	./Collision/b2PairManager.cpp \
	./Collision/b2CollidePoly.cpp \
	./Collision/b2CollideCircle.cpp \
	./Collision/b2BroadPhase.cpp \
	./Contrib/b2Polygon.cpp \
	./Contrib/b2Triangle.cpp

FLOAT_OBJECTS= $(addprefix Gen/float/,$(SOURCES:.cpp=.o))

Gen/float/%.o:		%.cpp
	mkdir -p $(dir $@)
	c++ $(CXXFLAGS) -c -o $@ $<


Gen/float/libbox2d.a:	$(FLOAT_OBJECTS)
	ar cr $@ $^
	ranlib $@ 


FIXED_OBJECTS= $(addprefix Gen/fixed/,$(SOURCES:.cpp=.o)) 

Gen/fixed/%.o:		%.cpp
	@mkdir -p $(dir $@)
	c++ $(CXXFLAGS) -DTARGET_FLOAT32_IS_FIXED -c -o $@ $^


Gen/fixed/libbox2d.a:	$(FIXED_OBJECTS)
	ar cr $@ $^
	ranlib $@ 


NDS_OBJECTS= $(addprefix Gen/nds-fixed/,$(SOURCES:.cpp=.o)) 

Gen/nds-fixed/%.o:		%.cpp
	@mkdir -p $(dir $@)
	arm-eabi-g++ -g -O2 -fomit-frame-pointer -ffast-math \
		-march=armv5te -mtune=arm946e-s -mthumb-interwork \
		-DARM9 -fno-rtti -fno-exceptions -c \
		-DTARGET_FLOAT32_IS_FIXED -DTARGET_IS_NDS \
		-I$(DEVKITPRO)/libnds/include \
		-o $@ $^

Gen/nds-fixed/lib/libbox2d.a:	$(NDS_OBJECTS)
	@mkdir -p $(dir $@)
	arm-eabi-ar cr $@ $^
	arm-eabi-ranlib $@ 


NDS_OBJECTS= $(addprefix Gen/nds-float/,$(SOURCES:.cpp=.o))

Gen/nds-float/%.o:		%.cpp
	@mkdir -p $(dir $@)
	arm-eabi-g++ -g -O2 -fomit-frame-pointer -ffast-math \
		-march=armv5te -mtune=arm946e-s -mthumb-interwork \
		-DARM9 -fno-rtti -fno-exceptions -c \
		-DTARGET_IS_NDS \
		-I$(DEVKITPRO)/libnds/include \
		-o $@ $^

Gen/nds-float/lib/libbox2d.a:	$(NDS_OBJECTS)
	@mkdir -p $(dir $@)
	arm-eabi-ar cr $@ $^
	arm-eabi-ranlib $@ 


