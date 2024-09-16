lab1: lab1.cpp
	g++ -g -c lab1.cpp utils.cpp render.cpp
	g++ lab1.o utils.o render.o -o lab1 -lsfml-graphics -lsfml-window -lsfml-system

lab1_student: lab1_student.cpp
	g++ -g -c lab1_student.cpp utils.cpp render.cpp
	g++ lab1_student.o utils.o render.o -o lab1_student -lsfml-graphics -lsfml-window -lsfml-system

debug_app: lab1.cpp
	g++ -g -O0 -fsanitize=address,undefined -c lab1.cpp  utils.cpp render.cpp
	g++ -g -O0 -fsanitize=address,undefined lab1.o utils.o render.o -o debug_app -lsfml-graphics -lsfml-window -lsfml-system

clean:
	rm *.o lab1

