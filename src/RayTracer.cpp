// The main ray tracer.

#pragma warning (disable: 4786)

#include "RayTracer.h"
#include "scene/light.h"
#include "scene/material.h"
#include "scene/ray.h"
#include "scene/kdTree.h"

#include "parser/Tokenizer.h"
#include "parser/Parser.h"

#include "ui/TraceUI.h"
#include <cmath>
#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>
#include <string.h> // for memset

#include <iostream>
#include <fstream>
#include <math.h>
#include <omp.h>

using namespace std;
extern TraceUI* traceUI;

// Use this variable to decide if you want to print out
// debugging messages.  Gets set in the "trace single ray" mode
// in TraceGLWindow, for example.
bool debugMode = true;

// Trace a top-level ray through pixel(i,j), i.e. normalized window coordinates (x,y),
// through the projection plane, and out into the scene.  All we do is
// enter the main ray-tracing method, getting things started by plugging
// in an initial ray weight of (0.0,0.0,0.0) and an initial recursion depth of 0.

glm::dvec3 RayTracer::trace(double x, double y)
{
	// Clear out the ray cache in the scene for debugging purposes,
	if (TraceUI::m_debug)
	{
		scene->clearIntersectCache();		
	}

	ray r(glm::dvec3(0,0,0), glm::dvec3(0,0,0), glm::dvec3(1,1,1), ray::VISIBILITY);
	scene->getCamera().rayThrough(x,y,r);
	double threshold = traceUI->getThreshold();
	glm::dvec3 ret = traceRay(r, glm::dvec3(1, 1, 1), traceUI->getDepth(), threshold);
	ret = glm::clamp(ret, 0.0, 1.0);
	return ret;

	// // ray r(glm::dvec3(0,0,0), glm::dvec3(0,0,0), glm::dvec3(1,1,1), ray::VISIBILITY);
	// Camera c = scene->getCamera();

	// // scene->getCamera().rayThrough(x,y,r);
	// int numSamples = traceUI->aaSwitch() ? traceUI->getSuperSamples() : 1;
	// double threshold = traceUI->getThreshold();
	
	// double offset = .5 / numSamples;
	// // std::cout << offset << std::endl;
	// glm::dvec3 ret = glm::dvec3(0, 0, 0);
	// for(int i = 0; i < numSamples; i++) {
	// 	for(int j = 0; j < numSamples; j++) {
	// 		ray r(glm::dvec3(0,0,0), glm::dvec3(0,0,0), glm::dvec3(1,1,1), ray::VISIBILITY);
	// 		double xi = x - offset - i / numSamples;
	// 		double yj = y - offset - j / numSamples;
	// 		std::cout << xi << " " << yj << std::endl;
	// 		glm::dvec3 dir = glm::normalize(c.getLook() + xi * c.getU() + yj * c.getV());
	// 		r.setPosition(c.getEye());
	// 		r.setDirection(dir);
	// 		ret += traceRay(r, glm::dvec3(1, 1, 1), traceUI->getDepth(), threshold);
	// 	}
	// }
	// ret /= (numSamples * numSamples);
	// ret = glm::clamp(ret, 0.0, 1.0);
	// return ret;
}

glm::dvec3 RayTracer::tracePixel(int i, int j)
{
	glm::dvec3 col(0,0,0);

	if( ! sceneLoaded() ) return col;

	double x = double(i)/double(buffer_width);
	double y = double(j)/double(buffer_height);

	unsigned char *pixel = buffer.data() + ( i + j * buffer_width ) * 3;

	int numSamples = traceUI->aaSwitch() ? traceUI->getSuperSamples() : 1;

	double offset = .5 / numSamples;
	// std::cout << offset << std::endl;
	for(int a = 0; a < numSamples; a++) {
		for(int b = 0; b < numSamples; b++) {
			double xa = (i + offset + a * offset * 2) / double(buffer_width);
			double yb = (j + offset + b * offset * 2) / double(buffer_height);
			col += trace(xa, yb);
		}
	}
	col /= numSamples * numSamples;

	pixel[0] = (int)( 255.0 * col[0]);
	pixel[1] = (int)( 255.0 * col[1]);
	pixel[2] = (int)( 255.0 * col[2]);
	return col;
}

bool tirOccurs(double n_r, glm::dvec3 n, glm::dvec3 i) {
	return 1 - n_r * n_r * (1 - std::pow(glm::dot(n, i), 2)) < 0;

}

#define VERBOSE 0

// Do recursive ray tracing!  You'll want to insert a lot of code here
// (or places called from here) to handle reflection, refraction, etc etc.
glm::dvec3 RayTracer::traceRay(ray& r, const glm::dvec3& thresh, int depth, double& t )
{
	isect i;
	glm::dvec3 colorC;
#if VERBOSE
	std::cerr << "== current depth: " << depth << std::endl;
#endif
	// 272638
	if (depth < 0) {
		return colorC;
	}
	// if(glm::length(r.getAtten()) <= )
	// if(glm::length(thresh) <= traceUI->getThreshold()) {
	// 	// std::cout << "below threshold" << std::endl;
	// 	return colorC;
	// }
	if(scene->intersect(r, i)) {
		// YOUR CODE HERE

		// An intersection occurred!  We've got work to do.  For now,
		// this code gets the material for the surface that was intersected,
		// and asks that material to provide a color for the ray.

		// This is a great place to insert code for recursive ray tracing.
		// Instead of just returning the result of shade(), add some
		// more steps: add in the contributions from reflected and refracted
		// rays.


		const Material& m = i.getMaterial();
		glm::dvec3 q = r.at(i.getT());
		glm::dvec3 normalVec = i.getN();


		glm::dvec3 reflectVec = glm::normalize(r.getDirection() - 2 * glm::dot(normalVec, r.getDirection()) * normalVec);
		ray reflectRay(q, reflectVec, glm::dvec3(1, 1, 1), ray::REFLECTION);
		reflectRay.setPosition(reflectRay.at(RAY_EPSILON));
		colorC = m.shade(scene.get(), r, i);
		if(glm::length(colorC) < t) {
			return colorC;
		}
		colorC += m.kr(i) * traceRay(reflectRay, thresh, depth - 1, t);

		glm::dvec3 i_vec = -r.getDirection();
		double n_i;
		double n_t;
		if (glm::dot(i_vec, normalVec) > 0) {
			n_i = 1;
			n_t = m.index(i);
		} else {
			n_i = m.index(i);
			n_t = 1;
			normalVec = -normalVec;
		}
		double n_r = n_i / n_t;
		if(glm::length(m.kt(i)) > 0.0 && !tirOccurs(n_r, normalVec, i_vec)) {
			glm::dvec3 t_vec = (n_r * glm::dot(normalVec, i_vec) - sqrt(1 - n_r * n_r * (1 - std::pow(glm::dot(normalVec, i_vec), 2)))) * normalVec - n_r * i_vec;
			t_vec = glm::normalize(t_vec);
			ray refractRay(q, t_vec, glm::dvec3(1,1,1), ray::REFRACTION);
			refractRay.setPosition(refractRay.at(RAY_EPSILON));
			colorC += m.kt(i) * traceRay(refractRay, thresh, depth - 1, t);
		}

	} else {
		// No intersection.  This ray travels to infinity, so we color
		// it according to the background color, which in this (simple) case
		// is just black.
		//
		// FIXME: Add CubeMap support here.
		// TIPS: CubeMap object can be fetched from traceUI->getCubeMap();
		//       Check traceUI->cubeMap() to see if cubeMap is loaded
		//       and enabled.

		if(traceUI->cubeMap()) {
			colorC = traceUI->getCubeMap()->getColor(r);
		} else {
			colorC = glm::dvec3(0, 0, 0);
		}
	}
#if VERBOSE
	std::cerr << "== depth: " << depth+1 << " done, returning: " << colorC << std::endl;
#endif
	return colorC;
}

RayTracer::RayTracer()
	: scene(nullptr), buffer(0), thresh(0), buffer_width(0), buffer_height(0), m_bBufferReady(false)
{
}

RayTracer::~RayTracer()
{
}

void RayTracer::getBuffer( unsigned char *&buf, int &w, int &h )
{
	buf = buffer.data();
	w = buffer_width;
	h = buffer_height;
}

double RayTracer::aspectRatio()
{
	return sceneLoaded() ? scene->getCamera().getAspectRatio() : 1;
}

bool RayTracer::loadScene(const char* fn)
{
	ifstream ifs(fn);
	if( !ifs ) {
		string msg( "Error: couldn't read scene file " );
		msg.append( fn );
		traceUI->alert( msg );
		return false;
	}

	// Strip off filename, leaving only the path:
	string path( fn );
	if (path.find_last_of( "\\/" ) == string::npos)
		path = ".";
	else
		path = path.substr(0, path.find_last_of( "\\/" ));

	// Call this with 'true' for debug output from the tokenizer
	Tokenizer tokenizer( ifs, false );
	Parser parser( tokenizer, path );
	try {
		scene.reset(parser.parseScene());
	}
	catch( SyntaxErrorException& pe ) {
		traceUI->alert( pe.formattedMessage() );
		return false;
	} catch( ParserException& pe ) {
		string msg( "Parser: fatal exception " );
		msg.append( pe.message() );
		traceUI->alert( msg );
		return false;
	} catch( TextureMapException e ) {
		string msg( "Texture mapping exception: " );
		msg.append( e.message() );
		traceUI->alert( msg );
		return false;
	}

	if (!sceneLoaded())
		return false;
	
	if(traceUI->kdSwitch()) {
		scene->createKdTree(traceUI->getMaxDepth(), traceUI->getLeafSize());
	}
	return true;
}

void RayTracer::traceSetup(int w, int h)
{
	size_t newBufferSize = w * h * 3;
	if (newBufferSize != buffer.size()) {
		bufferSize = newBufferSize;
		buffer.resize(bufferSize);
	}
	buffer_width = w;
	buffer_height = h;
	std::fill(buffer.begin(), buffer.end(), 0);
	m_bBufferReady = true;

	/*
	 * Sync with TraceUI
	 */

	threads = traceUI->getThreads();
	block_size = traceUI->getBlockSize();
	thresh = traceUI->getThreshold();
	samples = traceUI->getSuperSamples();
	aaThresh = traceUI->getAaThreshold();

	// You can add additional GUI functionality here as necessary
}

/*
 * RayTracer::traceImage
 *
 *	Trace the image and store the pixel data in RayTracer::buffer.
 *
 *	Arguments:
 *		w:	width of the image buffer
 *		h:	height of the image buffer
 *
 */
void RayTracer::traceImage(int w, int h)
{
	// Always call traceSetup before rendering anything.
	traceSetup(w,h);

	// YOUR CODE HERE
	// FIXME: Start one or more threads for ray tracing. 
	// OpenMP is probably best "bang for buck" time spent on this task
	//

	omp_set_num_threads(traceUI->getThreads());
	#pragma omp parallel
	{
		#pragma omp for nowait
		for(int i = 0; i < w; i++) {
			
			for(int j = 0; j < h; j++) {
				tracePixel(i, j);
			}
		}
	}
	
	

	// glm::dvec3 cameraOrigin = getScene().getCamera().getEye();
	// double dummy;
	// for(int i = 0; i < w; i++) {
	// 	for(int j = 0; j < h; j++) {
	// 		glm::dvec3 pixelPoint = getPixel(0, 0);
	// 		glm::dvec3 direction = glm::normalize(pixelPoint - cameraOrigin);
	// 		ray r = ray(cameraOrigin, direction, glm::dvec3(1, 1, 1));
	// 		setPixel(i, j, traceRay(r, glm::dvec3(1.0,1.0,1.0), 0, dummy));
	// 	}
	// }

	// glm::dvec3 test = getPixel(0, 0);
	// glm::dvec3 direction = glm::normalize(test - cameraOrigin);
	// double dummy;
	// traceRay(r, glm::dvec3(1.0,1.0,1.0), 0, dummy);

	// Alternatively traceImage can be executed asynchronously,
	//       i.e. returns IMMEDIATELY after working threads are launched.
	//
	//       An asynchronous traceImage lets the GUI update your results
	//       while rendering.
	//
	// This will require you to work with pthreads and queuing which is more involved, though
}


glm::dvec3 RayTracer::getPixel(int i, int j)
{
	unsigned char *pixel = buffer.data() + ( i + j * buffer_width ) * 3;
	return glm::dvec3((double)pixel[0]/255.0, (double)pixel[1]/255.0, (double)pixel[2]/255.0);
}

void RayTracer::setPixel(int i, int j, glm::dvec3 color)
{
	unsigned char *pixel = buffer.data() + ( i + j * buffer_width ) * 3;

	pixel[0] = (int)( 255.0 * color[0]);
	pixel[1] = (int)( 255.0 * color[1]);
	pixel[2] = (int)( 255.0 * color[2]);
}

