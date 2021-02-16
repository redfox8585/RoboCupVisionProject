CC = "g++"

PROJECT = ballDetection
SRC  = main.cpp

LIBS = `pkg-config opencv --cflags --libs`

$(PROJECT) : $(SRC)
	$(CC) $(SRC) -o $(PROJECT) $(LIBS)