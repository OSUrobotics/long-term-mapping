/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Brian Gerkey */

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>

#include "ros/ros.h"
#include "ros/console.h"
#include "map_server/image_loader.h"
#include "map_server/map_server.h"
#include "nav_msgs/MapMetaData.h"
#include "timemap_server/TimeLapseMap.h"
#include "timemap_server/TimeLapseOccGrid.h"
#include "yaml-cpp/yaml.h"
#include <tf/transform_broadcaster.h>


TimeMapServer::TimeMapServer(const std::string& fname, double res)
{
	std::vector <ImageData> mapfnames;
	timemap_server::TimeLapseMap time_map;
	ImageData new_map;
	unsigned int num_images;  
	double origin[3];
	int negate;
	double occ_th, free_th;
	MapMode mode = TRINARY;
	std::string frame_id;
	ros::NodeHandle private_nh("~");
	private_nh.param("frame_id", frame_id, std::string("map"));
	deprecated = (res != 0);


	if (deprecated) {
		ROS_ERROR("deprecated is bad..");
		exit(-1);
	}
	std::ifstream fin(fname.c_str());
	if (fin.fail()) {
		ROS_ERROR("Map_server could not open %s.", fname.c_str());
		exit(-1);
	}
#ifdef HAVE_NEW_YAMLCPP
	// The document loading process changed in yaml-cpp 0.5.
	YAML::Node doc = YAML::Load(fin);
	
#else
	YAML::Parser parser(fin);
	YAML::Node doc;
	parser.GetNextDocument(doc);
	

#endif
	try { 
		doc["resolution"] >> res; 
	} catch (YAML::InvalidScalar) { 
		ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
		exit(-1);
	}
	try { 
		doc["negate"] >> negate; 
	} catch (YAML::InvalidScalar) { 
		ROS_ERROR("The map does not contain a negate tag or it is invalid.");
		exit(-1);
	}
	try { 
		doc["occupied_thresh"] >> occ_th; 
	} catch (YAML::InvalidScalar) { 
		ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
		exit(-1);
	}
	try { 
		doc["free_thresh"] >> free_th; 
	} catch (YAML::InvalidScalar) { 
		ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
		exit(-1);
	}
	try { 
		std::string modeS = "";
		doc["mode"] >> modeS;

		if(modeS=="trinary")
			mode = TRINARY;
		else if(modeS=="scale")
			mode = SCALE;
		else if(modeS=="raw")
			mode = RAW;
		else{
			ROS_ERROR("Invalid mode tag \"%s\".", modeS.c_str());
			exit(-1);
		}
	} catch (YAML::Exception) { 
		ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary");
		mode = TRINARY;
	}
	try { 
		doc["origin"][0] >> origin[0]; 
		doc["origin"][1] >> origin[1]; 
		doc["origin"][2] >> origin[2]; 
	} catch (YAML::InvalidScalar) { 
		ROS_ERROR("The map does not contain an origin tag or it is invalid.");
		exit(-1);
	}


	try {
		num_images = doc["images"].size();
		ROS_INFO("num: %d", num_images);
		for(unsigned int i=0; i < num_images; i++) {
			doc["images"][i]["file"]  >> new_map.file;
			doc["images"][i]["begin"]["sec"] >> new_map.begin.sec;
			doc["images"][i]["begin"]["nsec"] >> new_map.begin.nsec;
			doc["images"][i]["end"]["sec"]   >> new_map.end.sec;
			doc["images"][i]["end"]["nsec"]   >> new_map.end.nsec;

			// TODO: make this path-handling more robust
			if(new_map.file.size() == 0)
			{
				ROS_ERROR("The image tag cannot be an empty string.");
				exit(-1);
			}
			if(new_map.file[0] != '/')
			{
				// dirname can modify what you pass it
				char* fname_copy = strdup(fname.c_str());
				new_map.file = std::string(dirname(fname_copy)) + '/' + new_map.file;
				free(fname_copy);
			}

			mapfnames.push_back(new_map);
		}
	} catch (YAML::InvalidScalar) { 
		ROS_ERROR("The map does not contain an image tag or it is invalid.");
		exit(-1);
	}

	// Latched publisher for data
	map_pub = n.advertise<timemap_server::TimeLapseMap>("time_map", 1, true);
	// tf::TransformBroadcaster broadcaster;
	ros::Time stamp = ros::Time::now();

	// ros::Rate r(5);
	for(unsigned int i=0; i < num_images; i++){
		timemap_server::TimeLapseOccGrid tgrid;

		ROS_INFO("Loading map from image \"%s\"", mapfnames[i].file.c_str());
		map_server::loadMapFromFile(&map_resp_,mapfnames[i].file.c_str(),res,negate,occ_th,free_th, origin, mode);
		map_resp_.map.info.map_load_time = ros::Time::now();
		map_resp_.map.header.frame_id = frame_id;
		map_resp_.map.header.stamp = ros::Time::now();
		ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
						 map_resp_.map.info.width,
						 map_resp_.map.info.height,
						 map_resp_.map.info.resolution);
		// meta_data_message_ = map_resp_.map.info;

		// metadata_pub.publish( meta_data_message_ );

		tgrid.map   = map_resp_.map;
		tgrid.begin = mapfnames[i].begin;
		tgrid.end   = mapfnames[i].end;

		time_map.tmap.push_back(tgrid);

		// r.sleep();
	}
	ROS_INFO("publish!");
	map_pub.publish( time_map );
}

		/** Callback invoked when someone requests our service */
bool TimeMapServer::mapCallback(nav_msgs::GetMap::Request  &req,
								 nav_msgs::GetMap::Response &res )
{
	// request is empty; we ignore it

	// = operator is overloaded to make deep copy (tricky!)
	res = map_resp_;
	ROS_INFO("Sending map");

	return true;
}