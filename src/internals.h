/*
 *  This file is part of libqemu-cxx
 *  Copyright (C) 2021 Luc Michel
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _LIBQEMU_CXX_INTERNALS_
#define _LIBQEMU_CXX_INTERNALS_

#include <map>

#include "libqemu/libqemu.h"
#include "libqemu-cxx/libqemu-cxx.h"

namespace qemu {

template <typename T>
class LibQemuObjectCallback {
private:
    std::map<QemuObject*, T> m_cbs;

public:
    void register_cb(Object obj, T cb)
    {
        m_cbs[obj.get_qemu_obj()] = cb;
    }

    void clear(Object obj)
    {
        m_cbs.erase(obj.get_qemu_obj());
    }

    template <typename... Args>
    void call(QemuObject *obj, Args... args) const
    {
        if (m_cbs.find(obj) == m_cbs.end()) {
            return;
        }

        m_cbs.at(obj)(args...);
    }
};

class LibQemuInternals {
private:
    LibQemu &m_inst;
    LibQemuExports *m_exports;

    LibQemuObjectCallback<Cpu::EndOfLoopCallbackFn> m_cpu_end_of_loop_cbs;

public:
    LibQemuInternals(LibQemu &inst, LibQemuExports *exports)
        : m_inst(inst)
        , m_exports(exports)
    {}

    const LibQemuExports & exports() const { return *m_exports; };
    LibQemu & get_inst() { return m_inst; }

    LibQemuObjectCallback<Cpu::EndOfLoopCallbackFn>& get_cpu_end_of_loop_cb()
    {
        return m_cpu_end_of_loop_cbs;
    }
};

}

#endif
