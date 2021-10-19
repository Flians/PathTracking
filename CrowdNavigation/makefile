LDFLAGS=-L./LG -lopengl32 -lglu32 -lglut32
CFLAGS=-g -DDEBUG -DGLUT_DISABLE_ATEXIT_HACK
all:
	g++ PathFinding.cpp -c -o PathFinding.o
	g++ main.cpp -c -o main.o $(CFLAGS)
	g++ main.o PathFinding.o $(LDFLAGS) -o CrowdNavigation.exe

clean:
	del PathFinding.o main.o CrowdNavigation.exe