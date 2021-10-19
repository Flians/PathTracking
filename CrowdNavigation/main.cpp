#include "PathFinding.h"

int main(int argc, char *argv[])
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	glutInitWindowPosition(300, 300);
	glutCreateWindow("Flow Field Path Finding");
	Initial();
	glutDisplayFunc(PathFindDisplay);
	glutMouseFunc(MouseClick);
	glutMotionFunc(MouseMove);
	glutTimerFunc(40, TimeerFunc, 1);
	glutMainLoop();

	return 0;
}