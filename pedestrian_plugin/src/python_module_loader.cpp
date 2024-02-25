// Copyright (c) 2024  Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "pedestrian_plugin/python_module_loader.hpp"
#include "pedestrian_plugin/pedestrian_plugin_manager.hpp"

using namespace std::chrono_literals;

std::shared_ptr<PythonModuleLoader> global_python_loader = std::make_shared<PythonModuleLoader>();

PythonModuleLoader::PythonModuleLoader() {}

PythonModuleLoader::~PythonModuleLoader()
{
  // Release all loaded Python objects and finalize the Python Interpreter
  for (auto & pair : modules) {
    Py_DECREF(pair.second);
  }
  modules.clear();
}

void PythonModuleLoader::reset()
{
  if ((std::chrono::system_clock::now() - lastResetTime) < 1s) {
    return;
  }

  std::map<std::string, PyObject *> temp;
  for (auto & pair : modules) {
    PyObject * pReloadedModule = PyImport_ReloadModule(pair.second);
    if (pReloadedModule != nullptr) {
      Py_DECREF(pair.second);
      temp.insert({pair.first, pReloadedModule});
    } else {
      PyErr_Print();
    }
  }
  modules.clear();
  for (auto & pair : temp) {
    modules.insert({pair.first, pair.second});
  }
  lastResetTime = std::chrono::system_clock::now();
}

PyObject * PythonModuleLoader::getFunc(const std::string & moduleName, const std::string & funcName)
{
  PyObject * module = loadModule(moduleName);
  if (module == NULL) {
    return NULL;
  }

  PyObject * func = PyObject_GetAttrString(module, "onUpdate");
  if (func == NULL || !PyCallable_Check(func)) {
    Py_XDECREF(func);
  }
  return func;
}

PyObject * PythonModuleLoader::loadModule(const std::string & moduleName)
{
  if (!Py_IsInitialized()) {
    PyImport_AppendInittab("ros", &PyInit_ros);
    Py_Initialize();
  }

  auto it = modules.find(moduleName);
  if (it != modules.end()) {
    return it->second;
  }

  PyObject * pName = PyUnicode_DecodeFSDefault(moduleName.c_str());
  PyObject * pModule = PyImport_Import(pName);
  Py_DECREF(pName);

  if (pModule == nullptr) {
    PyErr_Print();
    return NULL;
  }

  modules[moduleName] = pModule;
  return pModule;
}

PyObject * PythonModuleLoader::getModule(const std::string & moduleName)
{
  auto it = modules.find(moduleName);
  if (it != modules.end()) {
    return it->second;
  }
  return nullptr;
}
