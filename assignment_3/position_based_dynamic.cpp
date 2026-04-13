#include "GL/glew.h"
#include "GL/freeglut.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include "Eigen/Eigen"
#include <filesystem>
#include <set>
using namespace Eigen;
using namespace std;

float xRot = 0.0f;
float yRot = 0.f;
float xTrans = 0;
float yTrans = 0;
float zTrans = 0;
int   ox;
int   oy;
int   buttonState;
float xRotLength = 0.0f;
float yRotLength = 0.0f;
bool stop = true;
float window_width = 1000;
float window_height = 1000;
struct objMesh {
	vector<Vector3d> positions;
	vector<Vector3i> faces;
	vector<Vector2i> edges;
	vector<Vector3d> predict_positions;
	vector<Vector3d> velocities;
	vector<double> masses;
	vector<double> areas;
	vector<double> lengths;
	Vector2i upper_corners;
	Vector3d upper_corner_positions[2];
} cloth;

double maxY = -1e32;
double maxX = -1e32;
double maxZ = -1e32;
double minY = 1e32;
double minX = 1e32;
double minZ = 1e32;

bool load_triMesh(const std::string& filename, objMesh& mesh) {
	ifstream ifs(filename);
	if (!ifs) {
		fprintf(stderr, "unable to read file %s\n", filename.c_str());
		ifs.close();
		exit(-1);
		return false;
	}
	char buffer[1024];
	string line = "";
	int nodeNumber = 0;
	int elementNumber = 0;

	while (getline(ifs, line)) {
		string key = line.substr(0, 2);
		if (key.length() <= 1) continue;
		stringstream ss(line.substr(2));
		if (key == "v ") {
			Vector3d vert;
			ss >> vert[0] >> vert[1] >> vert[2];

			if (vert[0] > maxX) {
				maxX = vert[0];
			}
			if (vert[1] > maxY) {
				maxY = vert[1];
			}
			if (vert[2] > maxZ) {
				maxZ = vert[2];
			}
			if (vert[0] < minX) {
				minX = vert[0];
			}
			if (vert[1] < minY) {
				minY = vert[1];
			}
			if (vert[2] < minZ) {
				minZ = vert[2];
			}
			mesh.positions.push_back(vert);
		}
		else if (key == "f ") {
			if (line.length() >= 1024) {
				printf("[WARN]: skip line due to exceed max buffer length (1024).\n");
				continue;
			}

			std::vector<string> fs;

			{
				string buf;
				stringstream ss(line);
				vector<string> tokens;
				while (ss >> buf)
					tokens.push_back(buf);

				for (size_t index = 3; index < tokens.size(); index += 1) {
					fs.push_back("f " + tokens[1] + " " + tokens[index - 1] + " " + tokens[index]);
				}
			}

			int uv0, uv1, uv2;

			for (const auto& f : fs) {
				memset(buffer, 0, sizeof(char) * 1024);
				std::copy(f.begin(), f.end(), buffer);

				int faceVertIndexX, faceVertIndexY, faceVertIndexZ;
				int faceNormalIndexX, faceNormalIndexY, faceNormalIndexZ;

				if (sscanf(buffer, "f %d/%d/%d %d/%d/%d %d/%d/%d", &faceVertIndexX, &uv0, &faceNormalIndexX,
					&faceVertIndexY, &uv1, &faceNormalIndexY,
					&faceVertIndexZ, &uv2, &faceNormalIndexZ) == 9) {

					faceVertIndexX -= 1;
					faceVertIndexY -= 1;
					faceVertIndexZ -= 1;
				}
				else if (sscanf(buffer, "f %d %d %d", &faceVertIndexX,
					&faceVertIndexY,
					&faceVertIndexZ) == 3) {
					faceVertIndexX -= 1;
					faceVertIndexY -= 1;
					faceVertIndexZ -= 1;
				}
				Vector3i inds;
				inds[0] = faceVertIndexX;
				inds[1] = faceVertIndexY;
				inds[2] = faceVertIndexZ;
				mesh.faces.push_back(inds);
			}


		}
	}


	set<pair<uint64_t, uint64_t>> SFEdges_set;
	for (const auto& cTri : mesh.faces)
	{
		for (int i = 0; i < 3; i++)
		{
			for (int i = 0; i < 3; i++)
			{
				if (SFEdges_set.find(pair<uint32_t, uint32_t>(cTri[1], cTri[0]))
					== SFEdges_set.end()
					&& SFEdges_set.find(pair<uint32_t, uint32_t>(cTri[0], cTri[1]))
					== SFEdges_set.end())
				{
					SFEdges_set.insert(pair<uint32_t, uint32_t>(cTri[0], cTri[1]));
				}
				if (SFEdges_set.find(pair<uint32_t, uint32_t>(cTri[2], cTri[1]))
					== SFEdges_set.end()
					&& SFEdges_set.find(pair<uint32_t, uint32_t>(cTri[1], cTri[2]))
					== SFEdges_set.end())
				{
					SFEdges_set.insert(pair<uint32_t, uint32_t>(cTri[1], cTri[2]));
				}
				if (SFEdges_set.find(pair<uint32_t, uint32_t>(cTri[0], cTri[2]))
					== SFEdges_set.end()
					&& SFEdges_set.find(pair<uint32_t, uint32_t>(cTri[2], cTri[0]))
					== SFEdges_set.end())
				{
					SFEdges_set.insert(pair<uint32_t, uint32_t>(cTri[2], cTri[0]));
				}
			}
		}
	}

	vector<pair<uint64_t, uint64_t>> tempEdge =
		vector<pair<uint64_t, uint64_t>>(SFEdges_set.begin(), SFEdges_set.end());

	for (int i = 0; i < SFEdges_set.size(); i++)
	{
		mesh.edges.push_back(Vector2i(tempEdge[i].first, tempEdge[i].second));
	}

	ifs.close();
	return true;
}


void OutputCloth(const string& outpath) {
	ofstream out(outpath);
	out << "s 1" << endl;
	for (auto& v : cloth.positions) {
		out << "v " << v[2] << " " << v[1] << " " << -v[0] << endl;
	}
	for (const auto& ind : cloth.faces) {
		out << "f " << ind[0] + 1 << " " << ind[1] + 1 << " " << ind[2] + 1 << endl;
	}

	out.close();
}

void init_cloth() {
	string input_path = string{ input_dir };
	load_triMesh(input_path+"/cloth.obj", cloth);
	//init velocities
	cloth.velocities.resize(cloth.positions.size(), Vector3d(0, 0, 0));
	//init masses
	cloth.masses.resize(cloth.positions.size(), 0);
	//init predict position
	cloth.predict_positions = cloth.positions;
	//init initial area
	cloth.areas.resize(cloth.faces.size());
	double density = 200;
	double thickness = 1e-3;
	for (int i = 0; i < cloth.faces.size(); i++) {
		Vector3d point0 = cloth.positions[cloth.faces[i][0]];
		Vector3d point1 = cloth.positions[cloth.faces[i][1]];
		Vector3d point2 = cloth.positions[cloth.faces[i][2]];

		Vector3d vec01 = point1 - point0;
		Vector3d vec02 = point2 - point0;
		cloth.areas[i] = 0.5 * (vec01.cross(vec02)).norm();

		double tri_mass = cloth.areas[i] * thickness * density;

		cloth.masses[cloth.faces[i][0]] = cloth.masses[cloth.faces[i][0]] + tri_mass / 3;
		cloth.masses[cloth.faces[i][1]] = cloth.masses[cloth.faces[i][1]] + tri_mass / 3;
		cloth.masses[cloth.faces[i][2]] = cloth.masses[cloth.faces[i][2]] + tri_mass / 3;
	}
	//init initial edge length
	for (int i = 0; i < cloth.edges.size(); i++) {
		Vector3d point0 = cloth.positions[cloth.edges[i][0]];
		Vector3d point1 = cloth.positions[cloth.edges[i][1]];
		double length = (point0 - point1).norm();
		cloth.lengths.push_back(length);
	}

	//find two upper corner
	for (int i = 0; i < cloth.positions.size(); i++) {
		Vector3d vert = cloth.positions[i];
		if (vert[0]<minX + 1e-6 && vert[1] > maxY - 1e-6) {
			cloth.upper_corners[0] = i;
			cloth.upper_corner_positions[0] = vert;
		}
		else if (vert[0] > maxX - 1e-6 && vert[1] > maxY - 1e-6) {
			cloth.upper_corners[1] = i;
			cloth.upper_corner_positions[1] = vert;
		}
	}
}


void update_cloth_edges() {
	double dt = 0.01;
	Vector3d gravity_force = Vector3d(0, -9.8, -3);
	//update predict positions
	for (int i = 0; i < cloth.positions.size(); i++) {
		//cloth.velocities[i] = ? ;
		//cloth.predict_positions[i] = ? ;
	}

	for (int k = 0; k < 10; k++) {
		//do the prejection
		for (int i = 0; i < cloth.edges.size(); i++) {
			Vector3d point0 = cloth.predict_positions[cloth.edges[i][0]];
			Vector3d point1 = cloth.predict_positions[cloth.edges[i][1]];

			double lambda_min = 0.9;
			double lambda_max = 1.1;
			//calculate the new position
			//...
		}

		//fix two corners
		cloth.predict_positions[cloth.upper_corners[0]] = cloth.upper_corner_positions[0];
		cloth.predict_positions[cloth.upper_corners[1]] = cloth.upper_corner_positions[1];
	}
	//update final positions
	for (int i = 0; i < cloth.positions.size(); i++) {
		//cloth.velocities[i] = ?;
		//cloth.positions[i] = ?;
	}
}

void draw_mesh3D()
{
	glEnable(GL_DEPTH_TEST);
	glLineWidth(1.5f);
	glColor3f(0.9f, 0.1f, 0.1f);
	const vector<Vector3i>& surf = cloth.faces;  //obj.faces;
	glBegin(GL_TRIANGLES);

	for (int j = 0; j < surf.size(); j++)
	{
		glVertex3f((cloth.positions[surf[j][0]][0]),
			(cloth.positions[surf[j][0]][1]),
			(cloth.positions[surf[j][0]][2]));
		glVertex3f((cloth.positions[surf[j][1]][0]),
			(cloth.positions[surf[j][1]][1]),
			(cloth.positions[surf[j][1]][2]));
		glVertex3f((cloth.positions[surf[j][2]][0]),
			(cloth.positions[surf[j][2]][1]),
			(cloth.positions[surf[j][2]][2]));
	}
	glEnd();

	glColor3f(0.9f, 0.9f, 0.9f);
	glLineWidth(0.1f);
	glBegin(GL_LINES);


	const vector<Vector2i>& edges = cloth.edges;
	for (int j = 0; j < edges.size(); j++)
	{
		glVertex3f((cloth.positions[edges[j][0]][0]),
			(cloth.positions[edges[j][0]][1]),
			(cloth.positions[edges[j][0]][2]));
		glVertex3f((cloth.positions[edges[j][1]][0]),
			(cloth.positions[edges[j][1]][1]),
			(cloth.positions[edges[j][1]][2]));

		glColor3f(0.9f, 0.9f, 0.9f);
		glLineWidth(0.1f);
	}
	glEnd();
}

void draw_Scene3D()
{
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glTranslatef(xTrans, yTrans, zTrans);
	glRotatef(xRot, 1.0f, 0.0f, 0.0f);
	glRotatef(yRot, 0.0f, 1.0f, 0.0f);
	draw_mesh3D();
	glPopMatrix();
	glutSwapBuffers();
}

void display() {
	update_cloth_edges();
	
	draw_Scene3D();
}

void init(void)
{
	init_cloth();
	GLenum err = glewInit();
	if (GLEW_OK != err)
	{
		std::cerr << "Error: " << glewGetErrorString(err) << std::endl;
	}
	std::cerr << "Status: Using GLEW " << glewGetString(GLEW_VERSION) << std::endl;
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glViewport(0, 0, window_width, window_height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (float)window_width / window_height, 10.1f, 500.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0.0f, 0.0f, -4.0f);
}

void idle_func()
{
	glutPostRedisplay();
}

void reshape_func(GLint width, GLint height)
{
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (float)width / height, 0.1, 500.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0.0f, 0.0f, -4.0f);

}

void keyboard_func(unsigned char key, int x, int y)
{
	if (key == 'w')
	{
		zTrans += .3f;
	}

	if (key == 's')
	{
		zTrans -= .3f;
	}

	if (key == 'a')
	{
		xTrans += .3f;
	}

	if (key == 'd')
	{
		xTrans -= .3f;
	}

	if (key == 'q')
	{
		yTrans -= .3f;
	}

	if (key == 'e')
	{
		yTrans += .3f;
	}

	if (key == ' ')
	{
		stop = !stop;
	}
	glutPostRedisplay();
}

void special_keyboard_func(int key, int x, int y)
{
	glutPostRedisplay();
}

void mouse_func(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN)
	{
		buttonState = 1;
	}
	else if (state == GLUT_UP)
	{
		buttonState = 0;
	}
	ox = x;
	oy = y;
	glutPostRedisplay();
}

void motion_func(int x, int y)
{
	float dx, dy;
	dx = (float)(x - ox);
	dy = (float)(y - oy);
	if (buttonState == 1)
	{
		xRot += dy / 5.0f;
		yRot += dx / 5.0f;
	}
	ox = x;
	oy = y;
	glutPostRedisplay();
}


void SpecialKey(GLint key, GLint x, GLint y)
{
	if (key == GLUT_KEY_DOWN){}
	if (key == GLUT_KEY_UP){}
	glutPostRedisplay();
}

int main(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutSetOption(GLUT_MULTISAMPLE, 16);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE);
	glutInitWindowSize(window_width, window_height);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("FEM");
	init();
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_MULTISAMPLE);
	glHint(GL_MULTISAMPLE_FILTER_HINT_NV, GL_NICEST);
	glutDisplayFunc(display);
	glutReshapeFunc(reshape_func);
	glutKeyboardFunc(keyboard_func);
	glutSpecialFunc(&SpecialKey);
	glutMouseFunc(mouse_func);
	glutMotionFunc(motion_func);
	glutIdleFunc(idle_func);
	glutMainLoop();
}
