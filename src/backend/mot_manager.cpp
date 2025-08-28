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

#include <string>
#include <iostream>
#include "mot_manager.h"


// --- MOTEntity -----------------------------------------------------------------
void MOTEntity::AddSeg(int seg_number, bool last_seg, const uint8_t* data, size_t len) {
	if(last_seg)
		last_seg_number = seg_number;

	if(segs.find(seg_number) != segs.end()) {
		return;
	}

	// copy data
	segs[seg_number] = seg_t(len);
	memcpy(&segs[seg_number][0], data, len);
	size += len;
}

bool MOTEntity::IsFinished() const {
	if(last_seg_number == -1)
		return false;

	// check if all segments are available
	for(int i = 0; i <= last_seg_number; i++)
		if(segs.find(i) == segs.end())
			return false;

	return true;
}

std::vector<uint8_t> MOTEntity::GetData() const {
	std::vector<uint8_t> result(size);
	size_t offset = 0;

	// concatenate all segments
	for (int i = 0; i <= last_seg_number; i++) {
		const seg_t& seg = segs.at(i);
		memcpy(&result[offset], &seg[0], seg.size());
		offset += seg.size();
	}

	return result;
}


// --- MOTObject -----------------------------------------------------------------
void MOTObject::AddSeg(MOT_Datatype dg_type, int seg_number, bool last_seg, const uint8_t* data, size_t len) {
	switch (dg_type) {
		case MOT_Datatype::HEADER:
			header.AddSeg(seg_number, last_seg, data, len);
			break;
		case MOT_Datatype::UNSCRAMBLED_BODY:
			body.AddSeg(seg_number, last_seg, data, len);
			break;
		case MOT_Datatype::UNCOMPRESSED_DIRECTORY:
		case MOT_Datatype::SCRAMBLED_BODY:
		case MOT_Datatype::COMPRESSED_DIRECTORY:
			break;
	}
}

bool MOTObject::ParseCheckHeader(MOT_FILE& target_file) {
	MOT_FILE file = target_file;
	std::vector<uint8_t> data = header.GetData();

	// parse/check header core
	if(data.size() < 7)
		return false;

	size_t body_size = (data[0] << 20) | (data[1] << 12) | (data[2] << 4) | (data[3] >> 4);
	size_t header_size = ((data[3] & 0x0F) << 9) | (data[4] << 1) | (data[5] >> 7);
	int content_type = (data[5] & 0x7F) >> 1;
	int content_sub_type = ((data[5] & 0x01) << 8) | data[6];

//	fprintf(stderr, "body_size: %5zu, header_size: %3zu, content_type: 0x%02X, content_sub_type: 0x%03X\n",
//			body_size, header_size, content_type, content_sub_type);

	if(header_size != header.GetSize())
		return false;

	bool header_update =
			content_type == MOT_FILE::CONTENT_TYPE_MOT_TRANSPORT &&
			content_sub_type == MOT_FILE::CONTENT_SUB_TYPE_HEADER_UPDATE;

	// abort, if neither none nor both conditions (header received/update) apply
	if(header_received != header_update)
		return false;

	if(!header_update) {
		// store core info
		file.body_size = body_size;
		file.content_type = content_type;
		file.content_sub_type = content_sub_type;
	}

	std::string old_content_name = file.content_name;
	std::string new_content_name;

    // parse/check header extension
	for(size_t offset = 7; offset < data.size();) {
		int pli = data[offset] >> 6;
		int param_id = data[offset] & 0x3F;
		offset++;

		// get parameter len
		size_t data_len;
		switch(pli) {
		case 0b00:
			data_len = 0;
			break;
		case 0b01:
			data_len = 1;
			break;
		case 0b10:
			data_len = 4;
			break;
		case 0b11:
			if(offset >= data.size())
				return false;
			bool ext = data[offset] & 0x80;
			data_len = data[offset] & 0x7F;
			offset++;

			if(ext) {
				if(offset >= data.size())
					return false;
				data_len = (data_len << 8) + data[offset];
				offset++;
			}
			break;
		}

		if (offset + data_len - 1 >= data.size())
			return false;

		// process parameter
		switch(param_id) {
        case 0x04:	// ExpireTime
            file.expire_time = data[offset]; // TODO not tested
            break;
		case 0x05:	// TriggerTime
			if(data_len < 4)
				return false;
			// TODO: not only distinguish between Now or not
			file.trigger_time_now = !(data[offset] & 0x80);
//			fprintf(stderr, "TriggerTime: %s\n", file.trigger_time_now ? "Now" : "(not Now)");
			break;
		case 0x0C:	// ContentName
			if(data_len == 0)
				return false;
			//file.content_name = CharsetTools::ConvertTextToUTF8(&data[offset + 1], data_len - 1, data[offset] >> 4, true, &file.content_name_charset);
            file.content_name = toUtf8StringUsingCharset ( (const char *)&data[offset + 1], (CharacterSet) (data[offset] >> 4), data_len - 1);
			new_content_name = file.content_name;
//			fprintf(stderr, "ContentName: '%s'\n", file.content_name.c_str());
			break;
        case 0x25:  // Category/SlideID
            file.category = data[offset];
            file.slide_id = data[offset+1];
            break;
		case 0x26:	// CategoryTitle
            file.category_title = std::string((char*) &data[offset], data_len);	// already UTF-8
			break;
		case 0x27:	// ClickThroughURL
			file.click_through_url = std::string((char*) &data[offset], data_len);	// already UTF-8
//			fprintf(stderr, "ClickThroughURL: '%s'\n", file.click_through_url.c_str());
			break;
		}
		offset += data_len;
	}

	if(!header_update) {
		// ensure actual header is processed only once
		header_received = true;
	} else {
		// ensure matching content name
		if(new_content_name != old_content_name)
			return false;
	}

	target_file = file;
	return true;
}

bool MOTObject::IsToBeShown() {
	// abort, if already shown
	if(shown)
		return false;

	// try to process finished header
	if(header.IsFinished()) {
		// parse/check MOT header
		bool result = ParseCheckHeader(result_file);
		header.Reset();	// allow for header updates
		if(!result)
			return false;
	}

	// abort, if incomplete/not yet triggered
	if(!header_received)
		return false;
	if(!body.IsFinished() || result_file.body_size != body.GetSize())
		return false;
	if(!result_file.trigger_time_now)
		return false;

	// add body data
	result_file.data = body.GetData();

	shown = true;
	return true;
}

// --- MOTDirectory -----------------------------------------------------------------
void MOTDirectory::AddSeg(MOT_Datatype dg_type, int seg_number, bool last_seg, const uint8_t* data, size_t len) {
	switch (dg_type) {
		case MOT_Datatype::UNCOMPRESSED_DIRECTORY:
			directory.AddSeg(seg_number, last_seg, data, len);
			break;
		case MOT_Datatype::UNSCRAMBLED_BODY:
		case MOT_Datatype::HEADER:
		case MOT_Datatype::SCRAMBLED_BODY:
		case MOT_Datatype::COMPRESSED_DIRECTORY:
			break;
	}

	if (directory.IsFinished() && headers.empty()) {
		std::vector<uint8_t> data = directory.GetData();
		if(data.size() < 7)
			return;

		// EN 301 234 Figure 30
		const uint32_t directory_size =
			((uint32_t)(data[0] & 0x7F) << 24) |
			((uint32_t)(data[1]) << 16) |
			((uint32_t)(data[2]) << 8) |
			((uint32_t)(data[3]));

		const uint16_t num_objects =
			((uint16_t)(data[4]) << 8) |
			((uint16_t)(data[5]));

		// 24bits of data carousel period
		// 1 bit rfu
		// 2 bit rfa
		// 13 bits segment size

		const uint16_t extension_length_bytes =
			((uint16_t)(data[11]) << 8) |
			((uint16_t)(data[12]));

		/*
		std::cerr << "MOT Directory " <<
			"size=" << directory_size << " " <<
			"n=" << num_objects << " " <<
			"extlen=" << extension_length_bytes << "\n"; */

		size_t data_ix = 13 + extension_length_bytes;

		// Followed by num_objects directory entries
		for (size_t dir_ix = 0; dir_ix < num_objects; dir_ix++) {
			const uint16_t transport_id =
				((uint16_t)(data[data_ix]) << 8) |
				((uint16_t)(data[data_ix+1]));

			/*
			std::cerr << " Directory " << dir_ix <<
				" tid=" << transport_id <<
				" at " << data_ix << "\n"; */

			data_ix += 2;

			size_t rem = data.size() - data_ix;

			try {
				auto result = ParseDirectoryEntry(data.data() + data_ix, rem);
				data_ix += result.bytes_consumed;
				headers[transport_id] = result.file;
			}
			catch (const std::out_of_range& ex) {
				std::cerr << "OUT OF RANGE " <<  ex.what() << "\n";
				break;
			}
		}

		/*
		std::cerr << "Parsed MOT Directory (" << headers.size() << "):\n";
		for (const auto& entry : headers) {
			std::cerr << "  " << entry.first << ": " << entry.second.content_name << "\n";
		} */
	}
}

MOTDirectory::ParseResult MOTDirectory::ParseDirectoryEntry(const uint8_t *data, size_t data_len) {
	MOT_FILE file;

	// parse/check header core
	if (data_len < 7)
		throw std::out_of_range("data_len < 7");

	size_t body_size = (data[0] << 20) | (data[1] << 12) | (data[2] << 4) | (data[3] >> 4);
	size_t header_size = ((data[3] & 0x0F) << 9) | (data[4] << 1) | (data[5] >> 7);
	int content_type = (data[5] & 0x7F) >> 1;
	int content_sub_type = ((data[5] & 0x01) << 8) | data[6];

	fprintf(stderr, "body_size: %5zu, header_size: %3zu, content_type: 0x%02X, content_sub_type: 0x%03X\n", body_size, header_size, content_type, content_sub_type);

	// store core info
	file.body_size = body_size;
	file.content_type = content_type;
	file.content_sub_type = content_sub_type;

    // parse/check header extension
	size_t offset = 7;

	while (offset < header_size) {
		int pli = data[offset] >> 6;
		int param_id = data[offset] & 0x3F;
		offset++;

		// get parameter len
		size_t param_len;
		switch(pli) {
			case 0:
				param_len = 0;
				break;
			case 1:
				param_len = 1;
				break;
			case 2:
				param_len = 4;
				break;
			case 3:
				if (offset >= data_len)
					throw std::out_of_range("data_len for param 3");
				bool ext = data[offset] & 0x80;
				param_len = data[offset] & 0x7F;
				offset++;

				if(ext) {
					if(offset >= data_len)
						throw std::out_of_range("data_len for param 0b11 ext");
					param_len = (param_len << 8) + data[offset];
					offset++;
				}
				break;
		}

		if(offset + param_len - 1 >= data_len)
			throw std::out_of_range("data_len param value");

		// process parameter
		switch(param_id) {
        case 0x04:	// ExpireTime
            file.expire_time = data[offset]; // TODO not tested
            break;
		case 0x05:	// TriggerTime
			if(param_len < 4)
				throw std::out_of_range("param_len param triggertime");
			// TODO: not only distinguish between Now or not
			file.trigger_time_now = !(data[offset] & 0x80);
//			fprintf(stderr, "TriggerTime: %s\n", file.trigger_time_now ? "Now" : "(not Now)");
			break;
		case 0x0C:	// ContentName
			if(param_len == 0)
				throw std::out_of_range("param_len param contentname");
			//file.content_name = CharsetTools::ConvertTextToUTF8(&data[offset + 1], param_len - 1, data[offset] >> 4, true, &file.content_name_charset);
            file.content_name = toUtf8StringUsingCharset ( (const char *)&data[offset + 1], (CharacterSet) (data[offset] >> 4), param_len - 1);
//			fprintf(stderr, "ContentName: '%s'\n", file.content_name.c_str());
			break;
        case 0x25:  // Category/SlideID
            file.category = data[offset];
            file.slide_id = data[offset+1];
            break;
		case 0x26:	// CategoryTitle
            file.category_title = std::string((char*) &data[offset], param_len);	// already UTF-8
			break;
		case 0x27:	// ClickThroughURL
			file.click_through_url = std::string((char*) &data[offset], param_len);	// already UTF-8
//			fprintf(stderr, "ClickThroughURL: '%s'\n", file.click_through_url.c_str());
			break;
		}
		offset += param_len;
	}

	return {file, offset};
}

// --- MOTManager -----------------------------------------------------------------
MOTManager::MOTManager(bool directory_mode) :
	directory_mode(directory_mode) {
	Reset();
}

void MOTManager::Reset() {
	directory_transport_id = -1;
	directory = MOTDirectory();
	directory_entities.clear();

	object_transport_id = -1;
	object = MOTObject();
}

bool MOTManager::ParseCheckDataGroupHeader(const std::vector<uint8_t>& dg, size_t& offset, MOT_Datatype& dg_type) {
	// parse/check Data Group header
	if(dg.size() < (offset + 2))
		return false;

	bool extension_flag = dg[offset] & 0x80;
	bool crc_flag = dg[offset] & 0x40;
	bool segment_flag = dg[offset] & 0x20;
	bool user_access_flag = dg[offset] & 0x10;
	dg_type = static_cast<MOT_Datatype>(dg[offset] & 0x0F);
	offset += 2 + (extension_flag ? 2 : 0);

	if(!crc_flag)
		return false;
	if(!segment_flag)
		return false;
	if(!user_access_flag)
		return false;

	return true;
}

bool MOTManager::ParseCheckSessionHeader(const std::vector<uint8_t>& dg, size_t& offset, bool& last_seg, int& seg_number, int& transport_id) {
	// parse/check session header
	if(dg.size() < (offset + 3))
		return false;

	last_seg = dg[offset] & 0x80;
	seg_number = ((dg[offset] & 0x7F) << 8) | dg[offset + 1];
	bool transport_id_flag = dg[offset + 2] & 0x10;
	size_t len_indicator = dg[offset + 2] & 0x0F;
	offset += 3;

	if(!transport_id_flag)
		return false;
	if(len_indicator < 2)
		return false;

	// handle transport ID
	if(dg.size() < (offset + len_indicator))
		return false;

	transport_id = (dg[offset] << 8) | dg[offset + 1];
	offset += len_indicator;

	return true;
}

bool MOTManager::ParseCheckSegmentationHeader(const std::vector<uint8_t>& dg, size_t& offset, size_t& seg_size) {
	// parse/check segmentation header (MOT)
	if(dg.size() < (offset + 2))
		return false;

	seg_size = ((dg[offset] & 0x1F) << 8) | dg[offset + 1];
	offset += 2;

	// compare announced/actual segment size
	if(seg_size != dg.size() - offset - CalcCRC::CRCLen)
		return false;

	return true;
}

bool MOTManager::HandleMOTDataGroup(const std::vector<uint8_t>& dg) {
	size_t offset = 0;

	// parse/check headers
	MOT_Datatype dg_type;
	bool last_seg;
	int seg_number;
	int transport_id;
	size_t seg_size;

	if(!ParseCheckDataGroupHeader(dg, offset, dg_type))
		return false;
	if(!ParseCheckSessionHeader(dg, offset, last_seg, seg_number, transport_id))
		return false;
	if(!ParseCheckSegmentationHeader(dg, offset, seg_size))
		return false;

	if (directory_mode) {
		if (directory_transport_id != transport_id) {
			directory = MOTDirectory();
			directory_transport_id = transport_id;
		}

		if (dg_type == MOT_Datatype::UNCOMPRESSED_DIRECTORY) {
			directory.AddSeg(dg_type, seg_number, last_seg, &dg[offset], seg_size);
		}
		else if (dg_type == MOT_Datatype::UNSCRAMBLED_BODY) {
			auto& entity = directory_entities[transport_id];
			entity.AddSeg(seg_number, last_seg, &dg[offset], seg_size);
		}
		else {
			return false;
		}
		return true;
	}
	else {
		if (dg_type != MOT_Datatype::HEADER && dg_type != MOT_Datatype::UNSCRAMBLED_BODY) {
			return false;
		}

		if (object_transport_id != transport_id) {
			object_transport_id = transport_id;
			object = MOTObject();
		}

		object.AddSeg(dg_type, seg_number, last_seg, &dg[offset], seg_size);

		// check if object shall be shown
		bool display = object.IsToBeShown();
		//	fprintf(stderr, "dg_type: %d, seg_number: %2d%s, transport_id: %5d, size: %4zu; display: %s\n",
		//			dg_type, seg_number, last_seg ? " (LAST)" : "", transport_id, seg_size, display ? "true" : "false");

		// if object shall be shown, update it
		return display;
	}
}

MOT_FILE MOTManager::GetFile() const {
	if (directory_mode) {
		throw new std::logic_error("Don't use GetFile in MOT directory mode");
	}
	return object.GetFile();
}

std::vector<MOT_FILE> MOTManager::GetAllFiles() const {
	const auto status = directory.GetStatus();
	if (status.is_finished)
	{
		std::vector<MOT_FILE> files;
		for (const auto& item : status.headers) {
			const auto transport_id = item.first;

			const auto obj_it = directory_entities.find(transport_id);
			if (obj_it == directory_entities.end()) {
				std::cerr << "MOTManager::GetAllFiles(): Body of " << transport_id << "missing\n";
				return {};
			}

			auto body = obj_it->second.GetData();

			std::cerr << "MOTManager::GetAllFiles(): Body of " << transport_id <<
				" size=" << body.size() << "\n";

			MOT_FILE file = item.second;
			file.data = std::move(body);
			files.emplace_back(std::move(file));
		}

		return files;
	}

	return {};
}

