/*
    DABlin - capital DAB experience
    Copyright (C) 2016-2018 Stefan Pöschel

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MOT_MANAGER_H_
#define MOT_MANAGER_H_

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <map>
#include <vector>

#include "charsets.h"
#include "tools.h"

enum class MOT_Datatype {
    HEADER = 3,
    UNSCRAMBLED_BODY = 4,
    SCRAMBLED_BODY = 5,
    UNCOMPRESSED_DIRECTORY = 6,
    COMPRESSED_DIRECTORY = 7
};


// --- MOT_FILE -----------------------------------------------------------------
struct MOT_FILE {
	std::vector<uint8_t> data;

	// from header core
	size_t body_size;
	int content_type;
	int content_sub_type;

	// from header extension
	std::string content_name;
	std::string content_name_charset;	
	std::string click_through_url;
	bool trigger_time_now;
    uint32_t expire_time = 0;
    uint8_t category = 0;
    uint8_t slide_id = 0;
    std::string category_title;

	static const int CONTENT_TYPE_IMAGE			= 0x02;
	static const int CONTENT_TYPE_MOT_TRANSPORT	= 0x05;
	static const int CONTENT_SUB_TYPE_JFIF			= 0x001;
	static const int CONTENT_SUB_TYPE_PNG			= 0x003;
	static const int CONTENT_SUB_TYPE_HEADER_UPDATE	= 0x000;

	MOT_FILE() :
		body_size(-1),
		content_type(-1),
		content_sub_type(-1),
        trigger_time_now(false)
	{}
};


typedef std::vector<uint8_t> seg_t;
typedef std::map<int,seg_t> segs_t;

// --- MOTEntity -----------------------------------------------------------------
class MOTEntity {
private:
	segs_t segs;
	int last_seg_number;
	size_t size;
public:
	MOTEntity() {Reset();}
	void Reset() {
		segs.clear();
		last_seg_number = -1;
		size = 0;
	}

	void AddSeg(int seg_number, bool last_seg, const uint8_t* data, size_t len);
	bool IsFinished() const;
	size_t GetSize() const {return size;}
	std::vector<uint8_t> GetData() const;
};


// --- MOTObject -----------------------------------------------------------------
class MOTObject {
private:
	MOTEntity header;
	MOTEntity body;
	bool header_received;
	bool shown;

	MOT_FILE result_file;

	bool ParseCheckHeader(MOT_FILE& target_file);
public:
	MOTObject(): header_received(false), shown(false) {}

	void AddSeg(MOT_Datatype dg_type, int seg_number, bool last_seg, const uint8_t* data, size_t len);
	bool IsToBeShown();
	MOT_FILE GetFile() const { return result_file;}

	struct Status {
		bool body_is_finished;
		size_t body_size;
	};

	Status GetStatus() const {
		return {body.IsFinished(), body.GetSize()};
	}
};

class MOTDirectory {
private:
	MOTEntity directory;
	std::map<int /* transportId */, MOT_FILE> headers;

	struct ParseResult {
		MOT_FILE file;
		size_t bytes_consumed;
	};

	ParseResult ParseDirectoryEntry(const uint8_t *data, size_t data_len);

public:
	MOTDirectory() {}

	void AddSeg(MOT_Datatype dg_type, int seg_number, bool last_seg, const uint8_t* data, size_t len);

	struct Status {
		bool is_finished;
		size_t size;
		const std::map<int /* transportId */, MOT_FILE>& headers;
	};

	Status GetStatus() const {
		return {directory.IsFinished(), directory.GetSize(), headers};
	}
};

// --- MOTManager -----------------------------------------------------------------
class MOTManager {
private:
	// Used in directory mode
	int directory_transport_id = -1;
	MOTDirectory directory;
	std::map<int /*transport_id*/, MOTEntity> directory_entities;

	// Used in header mode
	int object_transport_id = -1;
	MOTObject object;

	bool directory_mode;

	bool ParseCheckDataGroupHeader(const std::vector<uint8_t>& dg, size_t& offset, MOT_Datatype& dg_type);
	bool ParseCheckSessionHeader(const std::vector<uint8_t>& dg, size_t& offset, bool& last_seg, int& seg_number, int& transport_id);
	bool ParseCheckSegmentationHeader(const std::vector<uint8_t>& dg, size_t& offset, size_t& seg_size);
public:
	MOTManager(bool directory_mode = false);

	void Reset();
	bool HandleMOTDataGroup(const std::vector<uint8_t>& dg);
	MOT_FILE GetFile() const;

	std::vector<MOT_FILE> GetAllFiles() const;
};

#endif /* MOT_MANAGER_H_ */
