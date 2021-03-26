/*************************************************************************/
/*  gdnative.h                                                           */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2020 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2020 Godot Engine contributors (cf. AUTHORS.md).   */
/*                                                                       */
/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the       */
/* "Software"), to deal in the Software without restriction, including   */
/* without limitation the rights to use, copy, modify, merge, publish,   */
/* distribute, sublicense, and/or sell copies of the Software, and to    */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions:                                             */
/*                                                                       */
/* The above copyright notice and this permission notice shall be        */
/* included in all copies or substantial portions of the Software.       */
/*                                                                       */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                */
/*************************************************************************/

#ifndef GDNATIVE_H
#define GDNATIVE_H

#include "core/io/resource_loader.h"
#include "core/io/resource_saver.h"
#include "core/os/thread_safe.h"
#include "core/resource.h"
#include "core/io/config_file.h"

class GDNativeLibraryResourceLoader;
class GDNative;

class GDNativeLibrary : public Resource {
	GDCLASS(GDNativeLibrary, Resource);

	static Map<String, Vector<Ref<GDNative> > > loaded_libraries;

	friend class GDNativeLibraryResourceLoader;
	friend class GDNative;

	Ref<ConfigFile> config_file;

	String current_library_path;
	Vector<String> current_dependencies;

	bool singleton;
	bool load_once;
	String symbol_prefix;
	bool reloadable;

public:
	GDNativeLibrary();
	~GDNativeLibrary();

	virtual bool _set(const StringName &p_name, const Variant &p_property);
	virtual bool _get(const StringName &p_name, Variant &r_property) const;
	virtual void _get_property_list(List<PropertyInfo> *p_list) const;

	_FORCE_INLINE_ Ref<ConfigFile> get_config_file() { return config_file; }

	void set_config_file(Ref<ConfigFile> p_config_file);

	// things that change per-platform
	// so there are no setters for this
	_FORCE_INLINE_ String get_current_library_path() const {
		return current_library_path;
	}
	_FORCE_INLINE_ Vector<String> get_current_dependencies() const {
		return current_dependencies;
	}

	// things that are a property of the library itself, not platform specific
	_FORCE_INLINE_ bool should_load_once() const {
		return load_once;
	}
	_FORCE_INLINE_ bool is_singleton() const {
		return singleton;
	}
	_FORCE_INLINE_ String get_symbol_prefix() const {
		return symbol_prefix;
	}

	_FORCE_INLINE_ bool is_reloadable() const {
		return reloadable;
	}

	_FORCE_INLINE_ void set_load_once(bool p_load_once) {
		config_file->set_value("general", "load_once", p_load_once);
		load_once = p_load_once;
	}
	_FORCE_INLINE_ void set_singleton(bool p_singleton) {
		config_file->set_value("general", "singleton", p_singleton);
		singleton = p_singleton;
	}
	_FORCE_INLINE_ void set_symbol_prefix(String p_symbol_prefix) {
		config_file->set_value("general", "symbol_prefix", p_symbol_prefix);
		symbol_prefix = p_symbol_prefix;
	}

	_FORCE_INLINE_ void set_reloadable(bool p_reloadable) {
		config_file->set_value("general", "reloadable", p_reloadable);
		reloadable = p_reloadable;
	}

	static void _bind_methods();
};

class GDNative : public Reference {
	GDCLASS(GDNative, Reference);

	Ref<GDNativeLibrary> library;

	void *native_handle;

	bool initialized;

public:
	GDNative();
	~GDNative();

	static void _bind_methods();

	void set_library(Ref<GDNativeLibrary> p_library);
	Ref<GDNativeLibrary> get_library() const;

	bool is_initialized() const;

	bool initialize();
	bool terminate();

	Variant call_native(StringName p_native_call_type, StringName p_procedure_name, Array p_arguments = Array());

	Error get_symbol(StringName p_procedure_name, void *&r_handle, bool p_optional = true) const;
};

#endif // GDNATIVE_H
