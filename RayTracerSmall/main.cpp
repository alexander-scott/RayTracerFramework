#include <stdlib.h>
#include <cstdio>
#include <cmath>
#include <fstream>
#include <vector>
#include <iostream>
#include <cassert>
#include <filesystem>

// Open CL
//#define __NO_STD_VECTOR // Use cl::vector instead of STL version
//#include <CL/cl.hpp>
//#include <cstdlib>
//#include <string>
//#include <iterator>

// Windows only
#include <algorithm>
#include <sstream>
#include <string.h>

// Time precision
#include <chrono>

// Threading
#include <thread>

template<typename T>
class Vec3
{
public:
	T x, y, z;
	Vec3() : x(T(0)), y(T(0)), z(T(0)) {}
	Vec3(T xx) : x(xx), y(xx), z(xx) {}
	Vec3(T xx, T yy, T zz) : x(xx), y(yy), z(zz) {}

	Vec3& normalize()
	{
		T nor2 = length2();
		if (nor2 > 0) {
			T invNor = 1 / sqrt(nor2);
			x *= invNor, y *= invNor, z *= invNor;
		}
		return *this;
	}

	Vec3<T> operator * (const T &f) const { return Vec3<T>(x * f, y * f, z * f); }
	Vec3<T> operator * (const Vec3<T> &v) const { return Vec3<T>(x * v.x, y * v.y, z * v.z); }
	T dot(const Vec3<T> &v) const { return x * v.x + y * v.y + z * v.z; }
	Vec3<T> operator - (const Vec3<T> &v) const { return Vec3<T>(x - v.x, y - v.y, z - v.z); }
	Vec3<T> operator + (const Vec3<T> &v) const { return Vec3<T>(x + v.x, y + v.y, z + v.z); }
	Vec3<T>& operator += (const Vec3<T> &v) { x += v.x, y += v.y, z += v.z; return *this; }
	Vec3<T>& operator *= (const Vec3<T> &v) { x *= v.x, y *= v.y, z *= v.z; return *this; }
	Vec3<T> operator - () const { return Vec3<T>(-x, -y, -z); }
	T length2() const { return x * x + y * y + z * z; }
	T length() const { return sqrt(length2()); }

	friend std::ostream & operator << (std::ostream &os, const Vec3<T> &v)
	{
		os << "[" << v.x << " " << v.y << " " << v.z << "]";
		return os;
	}
};

typedef Vec3<float> Vec3f;

class Sphere
{
public:
	Vec3f center;                           /// position of the sphere
	float radius, radius2;                  /// sphere radius and radius^2
	Vec3f surfaceColor, emissionColor;      /// surface color and emission (light)
	float transparency, reflection;         /// surface transparency and reflectivity

	Sphere() {}

	Sphere(
		const Vec3f &c,
		const float &r,
		const Vec3f &sc,
		const float &refl = 0,
		const float &transp = 0,
		const Vec3f &ec = 0) :
		center(c), radius(r), radius2(r * r), surfaceColor(sc), emissionColor(ec),
		transparency(transp), reflection(refl)
	{ /* empty */
	}

	//[comment]
	// Compute a ray-sphere intersection using the geometric solution
	//[/comment]
	bool intersect(const Vec3f &rayorig, const Vec3f &raydir, float &t0, float &t1) const
	{
		Vec3f l = center - rayorig;
		float tca = l.dot(raydir);
		if (tca < 0) return false;
		float d2 = l.dot(l) - tca * tca;
		if (d2 > radius2) return false;
		float thc = sqrt(radius2 - d2);
		t0 = tca - thc;
		t1 = tca + thc;

		return true;
	}
};

struct DriverInfo {
	float framerate; // The frames per second of the video
	int width;
	int height;
	int duration; // The length in second of the video
	int maxRayDepth; // This variable controls the maximum recursion depth
	std::string folderName; // The name of the folder that the .PPM frames will be temporarily saved to

	float totFrames;
};

DriverInfo config;

#if defined __linux__ || defined __APPLE__
// "Compiled for Linux
#else
// Windows doesn't define these values by default, Linux does
#define M_PI 3.141592653589793
#define INFINITY 1e8
#endif

enum Axis { XAxis, YAxis, ZAxis };
const Vec3f ORIGIN = Vec3f(0.0, 0, -200);

// Start and end timers
std::chrono::time_point<std::chrono::system_clock> start;
std::chrono::time_point<std::chrono::system_clock> end;
std::chrono::duration<double> total_elapsed_time;

// Thead pool
static const int num_threads = 10;

// If true the code will go down the threading route
bool performThreading = true;

float mix(const float &a, const float &b, const float &mix)
{
	return b * mix + a * (1 - mix);
}

//[comment]
// This is the main trace function. It takes a ray as argument (defined by its origin
// and direction). We test if this ray intersects any of the geometry in the scene.
// If the ray intersects an object, we compute the intersection point, the normal
// at the intersection point, and shade this point using this information.
// Shading depends on the surface property (is it transparent, reflective, diffuse).
// The function returns a color for the ray. If the ray intersects an object that
// is the color of the object at the intersection point, otherwise it returns
// the background color.
//[/comment]
Vec3f trace(
	const Vec3f &rayorig,
	const Vec3f &raydir,
	const std::vector<Sphere> &spheres,
	const int &depth)
{
	//if (raydir.length() != 1) std::cerr << "Error " << raydir << std::endl;
	float tnear = INFINITY;
	const Sphere* sphere = NULL;

	// find intersection of this ray with the sphere in the scene
	for (unsigned i = 0; i < spheres.size(); ++i)
	{
		float t0 = INFINITY, t1 = INFINITY;

		if (spheres[i].intersect(rayorig, raydir, t0, t1))
		{
			if (t0 < 0)
			{
				t0 = t1;
			}

			if (t0 < tnear)
			{
				tnear = t0;
				sphere = &spheres[i];
			}
		}
	}

	// if there's no intersection return black or background color
	if (!sphere)
	{
		return Vec3f(2);
	}

	Vec3f surfaceColor = 0; // color of the ray/surfaceof the object intersected by the ray
	Vec3f phit = rayorig + raydir * tnear; // point of intersection
	Vec3f nhit = phit - sphere->center; // normal at the intersection point

	nhit.normalize(); // normalize normal direction
					  // If the normal and the view direction are not opposite to each other
					  // reverse the normal direction. That also means we are inside the sphere so set
					  // the inside bool to true. Finally reverse the sign of IdotN which we want
					  // positive.

	float bias = 1e-4; // add some bias to the point from which we will be tracing
	bool inside = false;

	if (raydir.dot(nhit) > 0)
	{
		nhit = -nhit, inside = true;
	}

	if ((sphere->transparency > 0 || sphere->reflection > 0) && depth < config.maxRayDepth)
	{
		float facingratio = -raydir.dot(nhit);

		// change the mix value to tweak the effect
		float fresneleffect = mix(pow(1 - facingratio, 3), 1, 0.1);

		// compute reflection direction (not need to normalize because all vectors
		// are already normalized)
		Vec3f refldir = raydir - nhit * 2 * raydir.dot(nhit);

		refldir.normalize();

		Vec3f reflection = trace(phit + nhit * bias, refldir, spheres, depth + 1);
		Vec3f refraction = 0;

		// if the sphere is also transparent compute refraction ray (transmission)
		if (sphere->transparency)
		{
			float ior = 1.1, eta = (inside) ? ior : 1 / ior; // are we inside or outside the surface?
			float cosi = -nhit.dot(raydir);
			float k = 1 - eta * eta * (1 - cosi * cosi);

			Vec3f refrdir = raydir * eta + nhit * (eta *  cosi - sqrt(k));
			refrdir.normalize();
			refraction = trace(phit - nhit * bias, refrdir, spheres, depth + 1);
		}

		// the result is a mix of reflection and refraction (if the sphere is transparent)
		surfaceColor = (
			reflection * fresneleffect +
			refraction * (1 - fresneleffect) * sphere->transparency) * sphere->surfaceColor;
	}
	else
	{
		// it's a diffuse object, no need to raytrace any further
		for (unsigned i = 0; i < spheres.size(); ++i)
		{
			if (spheres[i].emissionColor.x > 0)
			{
				// this is a light
				Vec3f transmission = 1;
				Vec3f lightDirection = spheres[i].center - phit;
				lightDirection.normalize();

				for (unsigned j = 0; j < spheres.size(); ++j)
				{
					if (i != j)
					{
						float t0, t1;
						if (spheres[j].intersect(phit + nhit * bias, lightDirection, t0, t1))
						{
							transmission = 0;
							break;
						}
					}
				}

				surfaceColor += sphere->surfaceColor * transmission *
					std::max(float(0), nhit.dot(lightDirection)) * spheres[i].emissionColor;
			}
		}
	}

	return surfaceColor + sphere->emissionColor;
}

//[comment]
// Main rendering function. We compute a camera ray for each pixel of the image
// trace it and return a color. If the ray hits a sphere, we return the color of the
// sphere at the intersection point, else we return the background color.
//[/comment]
void render(const std::vector<Sphere> &spheres, int iteration, int threadNumber)
{
	start = std::chrono::system_clock::now();

#ifdef _DEBUG 
	// Recommended Testing Resolution
	unsigned width = 640, height = 480;
#else
	// Recommended Production Resolution
	//unsigned width = 1920, height = 1080;
	unsigned width = 640, height = 480;
#endif

	Vec3f *image = new Vec3f[width * height], *pixel = image;
	float invWidth = 1 / float(width), invHeight = 1 / float(height);
	float fov = 30, aspectratio = width / float(height);
	float angle = tan(M_PI * 0.5 * fov / 180.);

	// Trace rays
	for (unsigned y = 0; y < height; ++y)
	{
		for (unsigned x = 0; x < width; ++x, ++pixel)
		{
			float xx = (2 * ((x + 0.5) * invWidth) - 1) * angle * aspectratio;
			float yy = (1 - 2 * ((y + 0.5) * invHeight)) * angle;

			Vec3f raydir(xx, yy, -1);
			raydir.normalize();
			*pixel = trace(Vec3f(0, 20, 0), raydir, spheres, 0); // The Vec3f on this line is where the camera is positioned
		}
	}

	// Save result to a PPM image (keep these flags if you compile under Windows)
	std::stringstream ss;
	ss << "./" << config.folderName << "/spheres" << iteration << ".ppm";
	std::string tempString = ss.str();
	char* filename = (char*)tempString.c_str();

	std::ofstream ofs(filename, std::ios::out | std::ios::binary);
	ofs << "P6\n" << width << " " << height << "\n255\n";

	for (unsigned i = 0; i < width * height; ++i)
	{
		ofs << (unsigned char)(std::min(float(1), image[i].x) * 255) <<
			(unsigned char)(std::min(float(1), image[i].y) * 255) <<
			(unsigned char)(std::min(float(1), image[i].z) * 255);
	}

	ofs.close();
	delete[] image;

	end = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed_time = end - start;
	total_elapsed_time += elapsed_time;

	if (threadNumber != 0)
	{
		std::cout << "Finished image render #" << iteration << " in " << elapsed_time.count() << " on thread: " << threadNumber << std::endl;
	}
	else
	{
		std::cout << "Finished image render #" << iteration << " in " << elapsed_time.count() << std::endl;
	}
}

Vec3f rotate_point(Vec3f o, float angle, Vec3f p, Axis axis)
{
	float s = sin(angle);
	float c = cos(angle);

	// translate point back to origin:
	p.x -= o.x;
	p.y -= o.y;
	p.z -= o.z;

	float xnew;
	float ynew;
	float znew;

	switch (axis)
	{
	case XAxis:
		// X-Axis rotation
		xnew = p.x;
		ynew = p.y * c - p.z * s;
		znew = p.y * s + p.z * c;
		break;

	case YAxis:
		// Y-Axis rotation
		xnew = p.z * s + p.x * c;
		ynew = p.y;
		znew = p.z * c - p.x * s;
		break;

	case ZAxis:
		// Z-Axis rotation
		xnew = p.x * c - p.y * s;
		ynew = p.x * s + p.y * c;
		znew = p.z;
		break;
	}

	// translate point back:
	p.x = xnew + o.x;
	p.y = ynew + o.y;
	p.z = znew + o.z;
	return p;
}

void SolarSystem(int start, int finish, int threadNumber)
{
	std::vector<Sphere> spheres;
	// Vector structure for Sphere (position, radius, surface color, reflectivity, transparency, emission color)

	if (!performThreading)
	{
		start = 0;
		finish = config.totFrames;
	}

	for (float r = start; r <= config.totFrames && r <= finish; r++)
	{
		// Sun
		spheres.push_back(Sphere(ORIGIN, 15, Vec3f(1, 0.27, 0.0), 1, 0.5));

		/*
		How to use rotate_point:
		First vector is the point to rotate around, aka the origin.
		Next value is essentially how fast do you want to rotate around this point. Equation is ((r / totalFrames) * (ROTATION_SPEED)).
		(ROTATION_SPEED should be a value from 1 to 10. 1 being very slow, 10 being very fast.)
		The next vector is the start point of the rotation. Or a better way to look at is the objects current position.
		The final enum is which axis you want the rotation to take place in. Can't currently do any arbitrary rotation.
		*/

		// Mercury
		Vec3f newPos = rotate_point(ORIGIN, ((r / config.totFrames) * (6)), Vec3f(20.00, 0.32, ORIGIN.z), YAxis);
		spheres.push_back(Sphere(newPos, 2, Vec3f(0.75, 0.75, 0.75), 1, 0.5));

		// Venus
		newPos = rotate_point(ORIGIN, ((r / config.totFrames) * (4)), Vec3f(-30.00, 0.32, ORIGIN.z), YAxis);
		spheres.push_back(Sphere(newPos, 3, Vec3f(0.83, 0.92, 0.82), 1, 0.5));

		// Earth
		newPos = rotate_point(ORIGIN, ((r / config.totFrames) * (3)), Vec3f(0, 0.32, ORIGIN.z + 40), YAxis);
		Sphere earth = Sphere(newPos, 3.5, Vec3f(0.63, 0.90, 0.94), 1, 0.5);
		spheres.push_back(earth);

		// Moon
		newPos = rotate_point(earth.center, ((r / config.totFrames) * (10)), Vec3f(earth.center.x, earth.center.y, earth.center.z + 5), YAxis);
		spheres.push_back(Sphere(newPos, 1, Vec3f(0.75, 0.75, 0.75), 1, 0.5));

		// Mars
		newPos = rotate_point(ORIGIN, ((r / config.totFrames) * (5)), Vec3f(0, 0.32, ORIGIN.z - 50.00), YAxis);
		spheres.push_back(Sphere(newPos, 3, Vec3f(1.00, 0.32, -20.36), 0.5, 0.5));

		if (performThreading)
		{
			render(spheres, r, threadNumber);
		}
		else
		{
			render(spheres, r, 0);
		}

		spheres.clear();
	}
}

void DoThreading()
{
	std::thread threadPool[num_threads];

	int difference = (config.totFrames) / num_threads;

	for (int i = 0; i < num_threads; ++i)
	{
		threadPool[i] = std::thread(SolarSystem, i * difference, i * difference + difference, i);
	}

	//Join the threads with the main thread
	for (int i = 0; i < num_threads; ++i)
	{
		threadPool[i].join();
	}
}

void CreateVideo()
{
	std::stringstream ss;
	ss << "ffmpeg -y -f image2 -r " << config.framerate << " -i ./" << config.folderName << "/spheres%d.ppm -b 600k ./out.mp4";

	start = std::chrono::system_clock::now();

	system(ss.str().c_str());

	end = std::chrono::system_clock::now();

	std::chrono::duration<double> elapsed_time = end - start;
	total_elapsed_time += elapsed_time;
	std::time_t end_time = std::chrono::system_clock::to_time_t(end);
	std::cout << "**********************" << std::endl;
	std::cout << "Finished video render in " << elapsed_time.count() << std::endl;
	std::cout << "**********************" << std::endl;
	std::cout << "**********************" << std::endl;
	std::cout << "Total Render Time: " << total_elapsed_time.count() << std::endl;
	std::cout << "**********************" << std::endl;

	system("out.mp4");
}

void CreateFolder()
{
	namespace fs = std::experimental::filesystem;
	std::stringstream ss;
	ss << "./" << config.folderName;

	if (!fs::exists(ss.str().c_str())) // Check if temp folder exists
	{
		fs::create_directory(ss.str().c_str()); // Create temp folder
	}
}

void RemoveFolder()
{
	namespace fs = std::experimental::filesystem;
	std::stringstream ss;
	ss << "./" << config.folderName;

	if (fs::exists(ss.str().c_str())) // Check if temp folder exists
	{
		fs::remove_all(ss.str().c_str()); // Remove temp folder
	}
}

void GetConfig()
{
	std::ifstream infile("./input.txt");
	std::string line;
	std::size_t found = line.find("framerate");

	while (std::getline(infile, line, '\n'))
	{
		if (line.compare(0, 9, "framerate") == 0)
		{
			line.erase(0, 11);
			int fr = std::stoi(line);
			config.framerate = fr;
		}
		else if (line.compare(0, 5, "width") == 0)
		{
			line.erase(0, 7);
			int width = std::stoi(line);
			config.width = width;
		}
		else if (line.compare(0, 6, "height") == 0)
		{
			line.erase(0, 8);
			int height = std::stoi(line);
			config.height = height;
		}
		else if (line.compare(0, 8, "duration") == 0)
		{
			line.erase(0, 10);
			config.duration = stoi(line);
		}
		else if (line.compare(0, 11, "maxRayDepth") == 0)
		{
			line.erase(0, 13);
			int mrd = std::stoi(line);
			config.maxRayDepth = mrd;
		}
		else if (line.compare(0, 10, "folderName") == 0)
		{
			line.erase(0, 12);
			config.folderName = line;
		}
	}

	if (config.folderName == "")
	{
		config.folderName = "Temp";
	}

	config.totFrames = config.framerate * config.duration;
}

//[comment]
// In the main function, we will create the scene which is composed of 5 spheres
// and 1 light (which is also a sphere). Then, once the scene description is complete
// we render that scene, by calling the render() function.
//[/comment]
int main(int argc, char **argv)
{
	// This sample only allows one choice per program execution. Feel free to improve upon this
	srand(13);

	GetConfig();

	CreateFolder();

	if (performThreading)
	{
		DoThreading();
	}
	else
	{
		SolarSystem(0, 0, 0);
	}

#ifdef _DEBUG
	CreateVideo(); // Remove this later
#else
	CreateVideo();
#endif 

	RemoveFolder();

	return 0;
}






/*
void BasicRender()
{
std::vector<Sphere> spheres;
// Vector structure for Sphere (position, radius, surface color, reflectivity, transparency, emission color)

spheres.push_back(Sphere(Vec3f(0.0, -10004, -20), 10000, Vec3f(0.20, 0.20, 0.20), 0, 0.0));
spheres.push_back(Sphere(Vec3f(0.0, 0, -20), 4, Vec3f(1.00, 0.32, 0.36), 1, 0.5)); // The radius paramter is the value we will change
spheres.push_back(Sphere(Vec3f(5.0, -1, -15), 2, Vec3f(0.90, 0.76, 0.46), 1, 0.0));
spheres.push_back(Sphere(Vec3f(5.0, 0, -25), 3, Vec3f(0.65, 0.77, 0.97), 1, 0.0));

// This creates a file, titled 1.ppm in the current working directory
render(spheres, 1);

}

void SimpleShrinking()
{
std::vector<Sphere> spheres;
// Vector structure for Sphere (position, radius, surface color, reflectivity, transparency, emission color)

for (int i = 0; i < 4; i++)
{
if (i == 0)
{
spheres.push_back(Sphere(Vec3f(0.0, -10004, -20), 10000, Vec3f(0.20, 0.20, 0.20), 0, 0.0));
spheres.push_back(Sphere(Vec3f(0.0, 0, -20), 4, Vec3f(1.00, 0.32, 0.36), 1, 0.5)); // The radius paramter is the value we will change
spheres.push_back(Sphere(Vec3f(5.0, -1, -15), 2, Vec3f(0.90, 0.76, 0.46), 1, 0.0));
spheres.push_back(Sphere(Vec3f(5.0, 0, -25), 3, Vec3f(0.65, 0.77, 0.97), 1, 0.0));

}
else if (i == 1)
{
spheres.push_back(Sphere(Vec3f(0.0, -10004, -20), 10000, Vec3f(0.20, 0.20, 0.20), 0, 0.0));
spheres.push_back(Sphere(Vec3f(0.0, 0, -20), 3, Vec3f(1.00, 0.32, 0.36), 1, 0.5)); // Radius--
spheres.push_back(Sphere(Vec3f(5.0, -1, -15), 2, Vec3f(0.90, 0.76, 0.46), 1, 0.0));
spheres.push_back(Sphere(Vec3f(5.0, 0, -25), 3, Vec3f(0.65, 0.77, 0.97), 1, 0.0));
}
else if (i == 2)
{
spheres.push_back(Sphere(Vec3f(0.0, -10004, -20), 10000, Vec3f(0.20, 0.20, 0.20), 0, 0.0));
spheres.push_back(Sphere(Vec3f(0.0, 0, -20), 2, Vec3f(1.00, 0.32, 0.36), 1, 0.5)); // Radius--
spheres.push_back(Sphere(Vec3f(5.0, -1, -15), 2, Vec3f(0.90, 0.76, 0.46), 1, 0.0));
spheres.push_back(Sphere(Vec3f(5.0, 0, -25), 3, Vec3f(0.65, 0.77, 0.97), 1, 0.0));
}
else if (i == 3)
{
spheres.push_back(Sphere(Vec3f(0.0, -10004, -20), 10000, Vec3f(0.20, 0.20, 0.20), 0, 0.0));
spheres.push_back(Sphere(Vec3f(0.0, 0, -20), 1, Vec3f(1.00, 0.32, 0.36), 1, 0.5)); // Radius--
spheres.push_back(Sphere(Vec3f(5.0, -1, -15), 2, Vec3f(0.90, 0.76, 0.46), 1, 0.0));
spheres.push_back(Sphere(Vec3f(5.0, 0, -25), 3, Vec3f(0.65, 0.77, 0.97), 1, 0.0));
}

render(spheres, i);
// Dont forget to clear the Vector holding the spheres.
spheres.clear();
}
}

void SmoothScaling()
{
std::vector<Sphere> spheres;
// Vector structure for Sphere (position, radius, surface color, reflectivity, transparency, emission color)

int totalFrames = VIDEO_FPS * VIDEO_LENGTH;

for (float r = 0; r <= totalFrames; r++)
{
// TODO: Rotate spheres around a point over each frame

// Moving sphere. (r / totalFrames) changes radius over time
spheres.push_back(Sphere(Vec3f(0.0, 0, -20),
r / totalFrames,
Vec3f(1.00, 0.32, 0.36),
1,
0.5));

// Static spheres
//spheres.push_back(Sphere(Vec3f(0.0, -10004, -20), 10000, Vec3f(0.20, 0.20, 0.20), 0, 0.0));
spheres.push_back(Sphere(Vec3f(5.0, -1, -15), 2, Vec3f(0.90, 0.76, 0.46), 1, 0.0));
spheres.push_back(Sphere(Vec3f(5.0, 0, -25), 3, Vec3f(0.65, 0.77, 0.97), 1, 0.0));

render(spheres, r);

std::cout << "Rendered and saved spheres" << r << ".ppm" << std::endl;

// Dont forget to clear the Vector holding the spheres.
spheres.clear();
}
}

*/
