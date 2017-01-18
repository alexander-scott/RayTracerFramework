#include <stdlib.h>
#include <cstdio>
#include <cmath>
#include <fstream>
#include <vector>
#include <iostream>
#include <cassert>
#include <filesystem>
#include <iterator>
#include <utility>

// Windows only
#include <algorithm>
#include <sstream>
#include <string.h>

// Time precision
#include <chrono>

// Threading
#include <thread>

#pragma region Classes / Structs / Enums

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

enum ObjectType { Sphere, Cube };

enum Axis { XAxis, YAxis, ZAxis };

class ObjectDetails {
public:
	float radius = 0;
	float radius2; // Radius ^2
	float depth = 0;
	float height = 0;
	float width = 0;

	ObjectDetails() { }

	ObjectDetails(
		const float &rad,
		const float &dep,
		const float &hei,
		const float &wid) :
		radius(rad), depth(dep), height(hei), width(wid), radius2(radius * radius)
	{ }
};

class Object
{
public:
	Vec3f center;                           /// position of the object
	ObjectType objectType;					/// type of object (sphere or cube)
	ObjectDetails objectDetails;			/// size of the object
	Vec3f surfaceColor, emissionColor;      /// surface color and emission (light)
	float transparency, reflection;         /// surface transparency and reflectivity

	Object() {}

	Object(
		const Vec3f &cent,
		const ObjectType &type,
		const ObjectDetails &details,
		const Vec3f &sc,
		const float &refl = 0,
		const float &transp = 0,
		const Vec3f &ec = 0) :
		center(cent), objectType(type), surfaceColor(sc), objectDetails(details),
		emissionColor(ec), transparency(transp), reflection(refl)
	{ /* empty */
	}

	//[comment]
	// Compute a ray-object intersection using the geometric solution
	//[/comment]
	bool intersect(const Vec3f &rayorig, const Vec3f &raydir, float &t0, float &t1) const
	{
		switch (objectType)
		{
		case Sphere:
		{
			Vec3f l = center - rayorig;
			float tca = l.dot(raydir); // ANGLE

			if (tca < 0)
			{
				return false;
			}

			float d2 = l.dot(l) - tca * tca;

			if (d2 > objectDetails.radius2)
			{
				return false;
			}

			float thc = sqrt(objectDetails.radius2 - d2);
			t0 = tca - thc;
			t1 = tca + thc;

			return true;
		}

		case Cube:
		{
			Vec3f cubeMin;
			cubeMin.x = (center.x - (objectDetails.width / 2));
			cubeMin.y = (center.y - (objectDetails.height / 2));
			cubeMin.z = (center.z - (objectDetails.depth / 2));

			Vec3f cubeMax;
			cubeMax.x = (center.x + (objectDetails.width / 2));
			cubeMax.y = (center.y + (objectDetails.height / 2));
			cubeMax.z = (center.z + (objectDetails.depth / 2));

			Vec3f dirfrac;

			// r.dir is unit direction vector of ray
			dirfrac.x = 1.0f / raydir.x;
			dirfrac.y = 1.0f / raydir.y;
			dirfrac.z = 1.0f / raydir.z;

			float t = (cubeMin.x - rayorig.x)*dirfrac.x;
			float t2 = (cubeMax.x - rayorig.x)*dirfrac.x;
			float t3 = (cubeMin.y - rayorig.y)*dirfrac.y;
			float t4 = (cubeMax.y - rayorig.y)*dirfrac.y;
			float t5 = (cubeMin.z - rayorig.z)*dirfrac.z;
			float t6 = (cubeMax.z - rayorig.z)*dirfrac.z;

			t0 = std::max(std::max(std::min(t, t2), std::min(t3, t4)), std::min(t5, t6));
			t1 = std::min(std::min(std::max(t, t2), std::max(t3, t4)), std::max(t5, t6));

			// if t0 < 0, ray (line) is intersecting AABB, but whole AABB is behing us
			if (t0 < 0)
			{
				return false;
			}

			// if t0 > t1, ray doesn't intersect AABB
			if (t0 > t1)
			{
				return false;
			}

			return true;
		}
		}

		return false;
	}
};

struct DriverInfo {
	bool doThreading; // If true the code will go down the threading route
	bool optimise; // If true the code will go down the route with screen space subdivision
	bool solarSystem; // If true the solar system scene will be displayed
	float framerate; // The frames per second of the video
	int width;
	int height;
	int duration; // The length in second of the video
	int maxRayDepth; // This variable controls the maximum recursion depth
	std::string folderName; // The name of the folder that the .PPM frames will be temporarily saved to
	int movementSpeed; // The movement speed of the solar system (1-10)

	float totFrames;
};

#pragma endregion

#if defined __linux__ || defined __APPLE__
// "Compiled for Linux
#else
// Windows doesn't define these values by default, Linux does
#define M_PI 3.141592653589793
#define INFINITY 1e8
#endif

// Location of the sun
const Vec3f ORIGIN = Vec3f(0.0, 0, -300);

// Start and end timers
std::chrono::time_point<std::chrono::system_clock> start;
std::chrono::time_point<std::chrono::system_clock> end;
std::chrono::time_point<std::chrono::system_clock> programStart;
std::chrono::time_point<std::chrono::system_clock> programComplete;
std::chrono::duration<double> total_elapsed_time;

// Thead pool
static const int num_threads = 10;

int totalThreadsCreated = 0;

// Program config
DriverInfo config;

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
	const std::vector<Object> &objects,
	const int &depth)
{
	//if (raydir.length() != 1) std::cerr << "Error " << raydir << std::endl;
	float tnear = INFINITY;
	const Object* object = NULL;

	// find intersection of this ray with the object in the scene
	for (unsigned i = 0; i < objects.size(); ++i)
	{
		float t0 = INFINITY, t1 = INFINITY;

		if (objects[i].intersect(rayorig, raydir, t0, t1))
		{
			if (t0 < 0)
			{
				t0 = t1;
			}

			if (t0 < tnear)
			{
				tnear = t0;
				object = &objects[i];
			}
		}
	}

	// if there's no intersection return black or background color
	if (!object)
	{
		return Vec3f(2);
	}

	Vec3f surfaceColor = 0; // color of the ray/surfaceof the object intersected by the ray
	Vec3f phit = rayorig + raydir * tnear; // point of intersection
	Vec3f nhit = phit - object->center; // normal at the intersection point

	nhit.normalize(); // normalize normal direction
					  // If the normal and the view direction are not opposite to each other
					  // reverse the normal direction. That also means we are inside the object so set
					  // the inside bool to true. Finally reverse the sign of IdotN which we want
					  // positive.

	float bias = 1e-4; // add some bias to the point from which we will be tracing
	bool inside = false;

	if (raydir.dot(nhit) > 0)
	{
		nhit = -nhit, inside = true;
	}

	if ((object->transparency > 0 || object->reflection > 0) && depth < config.maxRayDepth)
	{
		float facingratio = -raydir.dot(nhit);

		// change the mix value to tweak the effect
		float fresneleffect = mix(pow(1 - facingratio, 3), 1, 0.1);

		// compute reflection direction (not need to normalize because all vectors
		// are already normalized)
		Vec3f refldir = raydir - nhit * 2 * raydir.dot(nhit);

		refldir.normalize();

		Vec3f reflection = trace(phit + nhit * bias, refldir, objects, depth + 1);
		Vec3f refraction = 0;

		// if the object is also transparent compute refraction ray (transmission)
		if (object->transparency)
		{
			float ior = 1.1, eta = (inside) ? ior : 1 / ior; // are we inside or outside the surface?
			float cosi = -nhit.dot(raydir);
			float k = 1 - eta * eta * (1 - cosi * cosi);

			Vec3f refrdir = raydir * eta + nhit * (eta *  cosi - sqrt(k));
			refrdir.normalize();
			refraction = trace(phit - nhit * bias, refrdir, objects, depth + 1);
		}

		// the result is a mix of reflection and refraction (if the object is transparent)
		surfaceColor = (
			reflection * fresneleffect +
			refraction * (1 - fresneleffect) * object->transparency) * object->surfaceColor;
	}
	else
	{
		// it's a diffuse object, no need to raytrace any further
		for (unsigned i = 0; i < objects.size(); ++i)
		{
			if (objects[i].emissionColor.x > 0)
			{
				// this is a light
				Vec3f transmission = 1;
				Vec3f lightDirection = objects[i].center - phit;
				lightDirection.normalize();

				for (unsigned j = 0; j < objects.size(); ++j)
				{
					if (i != j)
					{
						float t0, t1;
						if (objects[j].intersect(phit + nhit * bias, lightDirection, t0, t1))
						{
							transmission = 0;
							break;
						}
					}
				}

				surfaceColor += object->surfaceColor * transmission *
					std::max(float(0), nhit.dot(lightDirection)) * objects[i].emissionColor;
			}
		}
	}

	return surfaceColor + object->emissionColor;
}

void RenderThreadedFunction(Vec3f *pixel, const std::vector<Object> &objects, int width, float startY, float endY, float invWidth, float invHeight, float angle, float aspectratio)
{
	for (unsigned y = startY; y < endY; y++)
	{
		for (unsigned x = 0; x < width; ++x)
		{
			float xx = (2 * ((x + 0.5) * invWidth) - 1) * angle * aspectratio;
			float yy = (1 - 2 * ((y + 0.5) * invHeight)) * angle;

			Vec3f raydir(xx, yy, -1);
			raydir.normalize();

			*(pixel + ((y * width) + x)) = trace(Vec3f(0, 20, 0), raydir, objects, 0); 
		}
	}
}

//[comment]
// Main rendering function. We compute a camera ray for each pixel of the image
// trace it and return a color. If the ray hits a object, we return the color of the
// object at the intersection point, else we return the background color.
//[/comment]
void RenderPixels(const std::vector<Object> &objects, int iteration, int threadNumber, std::thread threadPool[2])
{
	start = std::chrono::system_clock::now();

#ifdef _DEBUG 
	// Recommended Testing Resolution
	unsigned width = 640, height = 480;
#else
	// Recommended Production Resolution
	unsigned width = 1920, height = 1080;
	//unsigned width = 640, height = 480;
#endif

	Vec3f *image = new Vec3f[width * height], *pixel = image;
	float invWidth = 1 / float(width), invHeight = 1 / float(height);
	float fov = 30, aspectratio = width / float(height);
	float angle = tan(M_PI * 0.5 * fov / 180.);

	if (config.optimise)
	{
		int difference = height / 2;
		int count = 0;

		// Trace rays on two different threads. One for the top half of the screen the other for the bottom half.
		for (unsigned y = 0; y < height; y += difference, count++)
		{
			threadPool[count] = std::thread(RenderThreadedFunction, pixel, objects, width,
				y, y + difference, invWidth, invHeight, angle, aspectratio);
		}

		//Join the threads with the main thread
		for (int i = 0; i < 2; ++i)
		{
			threadPool[i].join();
		}
	}
	else
	{
		// Trace rays
		for (unsigned y = 0; y < height; ++y)
		{
			for (unsigned x = 0; x < width; ++x, ++pixel)
			{
				float xx = (2 * ((x + 0.5) * invWidth) - 1) * angle * aspectratio;
				float yy = (1 - 2 * ((y + 0.5) * invHeight)) * angle;

				Vec3f raydir(xx, yy, -1);
				raydir.normalize();

				*pixel = trace(Vec3f(0, 20, 0), raydir, objects, 0); // The Vec3f on this line is where the camera is positioned
			}
		}
	}

	// Save result to a PPM image (keep these flags if you compile under Windows)
	std::stringstream ss;
	ss << "./" << config.folderName << "/objects" << iteration << ".ppm";
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

std::vector<Object> GetCubes()
{
	std::vector<Object> objectPool;

	for (int i = -2; i < 2; i++)
	{
		for (int j = -3; j < 3; j++)
		{
			objectPool.push_back(Object(
				Vec3f(i * 15, (rand() % 50), -100 - j * 15),
				Cube,
				ObjectDetails(5, 5, 5, 5),
				Vec3f(0.63, 0.90, 0.94),
				1,
				0));
		}
	}

	return objectPool;
}

void SolarSystem(int start, int finish, int threadNumber)
{
	std::vector<Object> objects;

	if (!config.doThreading)
	{
		start = 0;
		finish = config.totFrames;
	}

	std::thread threadPool[2];
	totalThreadsCreated += 2;

	// OBJECT DEFINITION
	// (POSITION, TYPE, DETAILS(RADIUS, DEPTH, HEIGHT, WIDTH), SURFACECOLOUR(R, G, B), REFLECTION, TRANSPARENCY, EMISSIONCOLOUR)

	for (float r = start; r <= config.totFrames && r <= finish; r++)
	{
		// Sun
		objects.push_back(Object(ORIGIN, Sphere, ObjectDetails(15, 15, 15, 15), Vec3f(1, 0.64, 0.0), 1, 0, Vec3f(1, 0.64, 0.0)));

		/*
		How to use rotate_point:
		First vector is the point to rotate around, aka the origin.
		Next value is essentially how fast do you want to rotate around this point. Equation is ((r / totalFrames) * (ROTATION_SPEED)).
		(ROTATION_SPEED should be a value from 1 to 10. 1 being very slow, 10 being very fast.)
		The next vector is the start point of the rotation. Or a better way to look at is the objects current position.
		The final enum is which axis you want the rotation to take place in. Can't currently do any arbitrary rotation.
		*/

		// Mercury
		Vec3f newPos = rotate_point(ORIGIN, ((r / config.totFrames) * (1.607 * config.movementSpeed)), Vec3f(20.00, 0.32, ORIGIN.z), YAxis);
		objects.push_back(Object(newPos, Sphere, ObjectDetails(2, 2, 2, 2), Vec3f(0.75, 0.75, 0.75), 1, 0));

		// Venus
		newPos = rotate_point(ORIGIN, ((r / config.totFrames) * (1.174 * config.movementSpeed)), Vec3f(-30.00, 0.32, ORIGIN.z), YAxis);
		objects.push_back(Object(newPos, Sphere, ObjectDetails(3, 3, 3, 3), Vec3f(0.83, 0.92, 0.82), 1, 0));

		// Earth
		newPos = rotate_point(ORIGIN, ((r / config.totFrames) * (1 * config.movementSpeed)), Vec3f(0, 0.32, ORIGIN.z + 40), YAxis);
		Object earth = Object(newPos, Sphere, ObjectDetails(3.5, 3.5, 3.5, 3.5), Vec3f(0.63, 0.90, 0.94), 1, 0);
		objects.push_back(earth);

		// Moon
		newPos = rotate_point(earth.center, ((r / config.totFrames) * (5 * config.movementSpeed)), Vec3f(earth.center.x, earth.center.y, earth.center.z + 5), YAxis);
		objects.push_back(Object(newPos, Sphere, ObjectDetails(1, 1, 1, 1), Vec3f(0.75, 0.75, 0.75), 1, 0));

		// Mars
		newPos = rotate_point(ORIGIN, ((r / config.totFrames) * (0.802 * config.movementSpeed)), Vec3f(0, 0.32, ORIGIN.z - 50.00), YAxis);
		objects.push_back(Object(newPos, Sphere, ObjectDetails(3, 3, 3, 3), Vec3f(1.00, 0.32, 0), 1, 0));

		// Jupiter
		newPos = rotate_point(ORIGIN, ((r / config.totFrames) * (0.434 * config.movementSpeed)), Vec3f(70.00, 0.32, ORIGIN.z), YAxis);
		objects.push_back(Object(newPos, Sphere, ObjectDetails(8.5, 8.5, 8.5, 8.5), Vec3f(0.78, 0.56, 0.22), 1, 0));

		// Saturn
		newPos = rotate_point(ORIGIN, ((r / config.totFrames) * (0.323 * config.movementSpeed)), Vec3f(-90.00, 0.32, ORIGIN.z), YAxis);
		objects.push_back(Object(newPos, Sphere, ObjectDetails(8, 8, 8, 8), Vec3f(0.76, 0.69, 0.50), 1, 0));

		// Uranus
		newPos = rotate_point(ORIGIN, ((r / config.totFrames) * (0.228 * config.movementSpeed)), Vec3f(0, 0.32, ORIGIN.z + 110), YAxis);
		objects.push_back(Object(newPos, Sphere, ObjectDetails(5, 5, 5, 5), Vec3f(0.52, 0.80, 0.98), 1, 0));

		// Neptune
		newPos = rotate_point(ORIGIN, ((r / config.totFrames) * (0.182 * config.movementSpeed)), Vec3f(0, 0.32, ORIGIN.z - 130), YAxis);
		objects.push_back(Object(newPos, Sphere, ObjectDetails(5, 5, 5, 5), Vec3f(0.52, 0.80, 0.98), 1, 0));

		if (config.doThreading)
		{
			RenderPixels(objects, r, threadNumber, threadPool);
		}
		else
		{
			RenderPixels(objects, r, 0, threadPool);
		}

		objects.clear();
	}
}

void FallingCubes(int start, int finish, int threadNumber, std::vector<Object> objectPool)
{
	std::vector<Object> objects;

	if (!config.doThreading)
	{
		start = 0;
		finish = config.totFrames;

		objects = GetCubes();
	}

	std::thread threadPool[2];

	for (float r = start; r <= config.totFrames && r <= finish; r++)
	{
		for (int i = 0; i < objectPool.size(); i++)
		{
			objects.push_back(Object(Vec3f(objectPool[i].center.x, objectPool[i].center.y - ((r / config.totFrames) * (config.movementSpeed)),
				objectPool[i].center.z), objectPool[i].objectType, objectPool[i].objectDetails, objectPool[i].surfaceColor,
				objectPool[i].reflection, objectPool[i].transparency, objectPool[i].emissionColor));
		}

		if (config.doThreading)
		{
			RenderPixels(objects, r, threadNumber, threadPool);
		}
		else
		{
			RenderPixels(objects, r, 0, threadPool);
		}

		objects.clear();
	}
}

void DoThreading()
{
	std::thread threadPool[num_threads];
	std::vector<Object> objectPool;

	int difference = (config.totFrames) / num_threads;

	if (!config.solarSystem)
	{
		objectPool = GetCubes();
	}

	for (int i = 0; i < num_threads; ++i)
	{
		if (config.solarSystem)
		{
			threadPool[i] = std::thread(SolarSystem, i * difference, i * difference + difference, i);
		}
		else
		{
			threadPool[i] = std::thread(FallingCubes, i * difference, i * difference + difference, i, objectPool);
		}
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
	ss << "ffmpeg -y -f image2 -r " << config.framerate << " -i ./" << config.folderName << "/objects%d.ppm -b 600k ./out.mp4";

	start = std::chrono::system_clock::now();

	system(ss.str().c_str());

	end = std::chrono::system_clock::now();

	std::chrono::duration<double> elapsed_time = end - start;
	total_elapsed_time += elapsed_time;

	std::chrono::duration<double> elapsed_time_full = end - programStart;

	std::time_t end_time = std::chrono::system_clock::to_time_t(end);
	std::cout << "**********************" << std::endl;
	std::cout << "Finished video render in " << elapsed_time.count() << std::endl;
	std::cout << "**********************" << std::endl;
	std::cout << "**********************" << std::endl;
	std::cout << "Total Render Time: " << elapsed_time_full.count() << std::endl;
	std::cout << "**********************" << std::endl;

	std::cout << totalThreadsCreated;

	system("out.mp4");
}

void CreateFolder()
{
	namespace fs = std::experimental::filesystem;
	std::stringstream ss;
	ss << "./" << config.folderName;

	if (!fs::exists(ss.str().c_str())) // Check if temp folder doesn't exist
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
		if (line.compare(0, 11, "doThreading") == 0)
		{
			line.erase(0, 13);
			int thread = std::stoi(line);
			if (thread == 1)
			{
				config.doThreading = true;
			}
			else
			{
				config.doThreading = false;
			}
		}
		else if (line.compare(0, 8, "optimise") == 0)
		{
			line.erase(0, 10);
			int opt = std::stoi(line);
			if (opt == 1)
			{
				config.optimise = true;
			}
			else
			{
				config.optimise = false;
			}
		}
		else if (line.compare(0, 11, "solarSystem") == 0)
		{
			line.erase(0, 13);
			int ss = std::stoi(line);
			if (ss == 1)
			{
				config.solarSystem = true;
			}
			else
			{
				config.solarSystem = false;
			}
		}
		else if (line.compare(0, 9, "framerate") == 0)
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
		else if (line.compare(0, 13, "movementSpeed") == 0)
		{
			line.erase(0, 15);
			int ms = std::stoi(line);
			config.movementSpeed = ms;
		}
	}

	if (config.folderName == "")
	{
		config.folderName = "Temp";
	}

	config.totFrames = config.framerate * config.duration;
}

//[comment]
// In the main function, we will create the scene which is composed of 5 objects
// and 1 light (which is also a objects). Then, once the scene description is complete
// we render that scene, by calling the render() function.
//[/comment]
int main(int argc, char **argv)
{
	// This sample only allows one choice per program execution. Feel free to improve upon this
	srand(13);

	programStart = std::chrono::system_clock::now();

	GetConfig();

	CreateFolder();

	if (config.doThreading)
	{
		DoThreading();
	}
	else
	{
		if (config.solarSystem)
		{
			SolarSystem(0, 0, 0);
		}
		else
		{
			std::vector<Object> objectPool;
			FallingCubes(0, 0, 0, objectPool);
		}
	}

#ifdef _DEBUG
	CreateVideo(); // Remove this later
#else
	CreateVideo();
#endif 

	RemoveFolder();

	return 0;
}