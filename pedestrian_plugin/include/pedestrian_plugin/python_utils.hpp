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

#ifndef PEDESTRIAN_PLUGIN__PYTHON_UTILS_HPP_
#define PEDESTRIAN_PLUGIN__PYTHON_UTILS_HPP_

#include <Python.h>
#include <map>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

class PythonUtils
{
public:
  static void setDictItemAsFloat(PyObject * dict, std::string key, double value)
  {
    auto pValue = PyFloat_FromDouble(value);
    PyDict_SetItemString(dict, key.c_str(), pValue);
    Py_DECREF(pValue);
  }

  static double getDictItemAsDouble(PyObject * dict, std::string key, double default_value = 0.0)
  {
    auto pName = PyUnicode_FromString(key.c_str());
    auto obj = PyDict_GetItem(dict, pName);
    Py_DECREF(pName);
    if (obj == NULL) {
      return default_value;
    }
    auto d = PyFloat_AsDouble(obj);
    return d;
  }

  static PyObject* getDictItemAsDict(PyObject * dict, std::string key)
  {
    auto pName = PyUnicode_FromString(key.c_str());
    auto obj = PyDict_GetItem(dict, pName);
    Py_DECREF(pName);
    if (obj == NULL) {
      return NULL;
    }
    return obj;
  }

  // Utility function to convert Python Unicode object to std::string (UTF-8)
  static std::string PyUnicodeObject_ToStdString(PyObject * unicodeObj)
  {
    if (!unicodeObj || !PyUnicode_Check(unicodeObj)) {
      throw std::runtime_error("Provided object is not a Unicode string");
    }

    PyObject * tempBytes = PyUnicode_AsEncodedString(unicodeObj, "utf-8", "strict");
    if (!tempBytes) {
      throw std::runtime_error("Encoding to UTF-8 failed");
    }

    const char * utf8String = PyBytes_AsString(tempBytes);
    if (!utf8String) {
      Py_DECREF(tempBytes);
      throw std::runtime_error("Error converting to C string");
    }

    std::string result(utf8String);
    Py_DECREF(tempBytes);
    return result;
  }

  // debug function
  static void print_pyobject(PyObject * obj, rclcpp::Logger logger)
  {
    if (PyLong_Check(obj)) {
      // It's a long integer
      int64_t value = PyLong_AsLong(obj);
      RCLCPP_INFO(logger, "Integer: %ld", value);
    } else if (PyFloat_Check(obj)) {
      // It's a float
      double value = PyFloat_AsDouble(obj);
      RCLCPP_INFO(logger, "Float: %f", value);
    } else if (PyUnicode_Check(obj)) {
      // It's a Unicode string
      PyObject * tempBytes = PyUnicode_AsEncodedString(obj, "utf-8", "strict");
      if (tempBytes != NULL) {
        char * str = PyBytes_AsString(tempBytes);
        if (str != NULL) {
          RCLCPP_INFO(logger, "String: %s", str);
        }
        Py_DECREF(tempBytes);
      }
    } else if (PyList_Check(obj)) {
      // It's a list
      Py_ssize_t size = PyList_Size(obj);
      RCLCPP_INFO(logger, "List of size %zd: [", size);
      for (Py_ssize_t i = 0; i < size; i++) {
        PyObject * item = PyList_GetItem(obj, i);
        print_pyobject(item, logger);  // Recursive call to print each item
        if (i < size - 1) {
          RCLCPP_INFO(logger, ", ");
        }
      }
      RCLCPP_INFO(logger, "]");
    } else if (PyDict_Check(obj)) {
      // It's a dictionary
      RCLCPP_INFO(logger, "Dictionary: {");
      PyObject * key, * value;
      Py_ssize_t pos = 0;

      while (PyDict_Next(obj, &pos, &key, &value)) {
        print_pyobject(key, logger);
        RCLCPP_INFO(logger, ": ");
        print_pyobject(value, logger);
      }
      RCLCPP_INFO(logger, "}");
    } else if (PyTuple_Check(obj)) {
      // It's a tuple
      Py_ssize_t size = PyTuple_Size(obj);
      RCLCPP_INFO(logger, "Tuple of size %zd: (", size);
      for (Py_ssize_t i = 0; i < size; i++) {
        PyObject * item = PyTuple_GetItem(obj, i);  // Get item from tuple
        print_pyobject(item, logger);               // Recursive call to print each item
        if (i < size - 1) {
          RCLCPP_INFO(logger, ", ");
        }
      }
      RCLCPP_INFO(logger, ")");
    } else {
      // Object type is not handled in this example
      RCLCPP_INFO(logger, "Object type not handled in this example.");
    }
  }
};

#endif  // PEDESTRIAN_PLUGIN__PYTHON_UTILS_HPP_
