/*******************************************************************************
 * Copyright (c) 2024  Carnegie Mellon University
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

#ifndef PEDESTRIAN_PLUGIN_PYTHON_MODULE_LOADER_HPP_
#define PEDESTRIAN_PLUGIN_PYTHON_MODULE_LOADER_HPP_

#include <Python.h>
#include <string>
#include <chrono>
#include <map>

using namespace std::chrono_literals; 

class PythonModuleLoader {
private:
    std::map<std::string, PyObject*> modules;

public:
    PythonModuleLoader() {
    }

    ~PythonModuleLoader() {
        // Release all loaded Python objects and finalize the Python Interpreter
        reset();
    }

    bool canReset() {
        return (std::chrono::system_clock::now() - lastResetTime) > 1s;
    }

    void reset() {
        for (auto& pair : modules) {
            Py_DECREF(pair.second);
        }
        Py_Finalize();
        lastResetTime = std::chrono::system_clock::now();
    }

    PyObject* getFunc(const std::string & moduleName, const std::string & funcName)  {
        PyObject* module = this->loadModule(moduleName);

        if (module != NULL) {
            PyObject *func = PyObject_GetAttrString(module, "onUpdate");
            if (func == NULL || !PyCallable_Check(func)) {
                Py_XDECREF(func);
            }
            return func;
        }
        return NULL;
    }

    PyObject* loadModule(const std::string& moduleName) {
        if (!Py_IsInitialized()) {
            Py_Initialize();
        }

        PyObject* pName = PyUnicode_DecodeFSDefault(moduleName.c_str());
        PyObject* pModule = PyImport_Import(pName);
        Py_DECREF(pName);

        if (pModule == nullptr) {
            PyErr_Print();
            return NULL;
        }

        modules[moduleName] = pModule;
        return pModule;
    }

    PyObject* getModule(const std::string& moduleName) {
        auto it = modules.find(moduleName);
        if (it != modules.end()) {
            return it->second;
        }
        return nullptr;
    }

    std::chrono::time_point<std::chrono::system_clock> lastResetTime;
};

#endif  // PEDESTRIAN_PLUGIN_PYTHON_MODULE_LOADER_HPP_
