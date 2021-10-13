/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the QtCore module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "mainwidget.h"

#include <QMouseEvent>

#include <cmath>

MainWidget::~MainWidget()
{
    // Make sure the context is current when deleting the texture
    // and the buffers.
    makeCurrent();
    delete arrayBuf;
    delete indexBuf;
    doneCurrent();
}

//! [0]
void MainWidget::mousePressEvent(QMouseEvent *e)
{
    // Save mouse press position
    mousePressPosition = QVector2D(e->localPos());
}

void MainWidget::mouseReleaseEvent(QMouseEvent *e)
{
    // Mouse release position - mouse press position
    QVector2D diff = QVector2D(e->localPos()) - mousePressPosition;

    // Rotation axis is perpendicular to the mouse position difference
    // vector
    QVector3D n = QVector3D(diff.y(), diff.x(), 0.0).normalized();

    // Accelerate angular speed relative to the length of the mouse sweep
    qreal acc = diff.length() / 100.0;

    // Calculate new rotation axis as weighted sum
    rotationAxis = (rotationAxis * angularSpeed + n * acc).normalized();

    // Increase angular speed
    angularSpeed += acc;
}
//! [0]

//! [1]
void MainWidget::timerEvent(QTimerEvent *)
{
    // Decrease angular speed (friction)
    angularSpeed *= 0.99;

    // Stop rotation when speed goes below threshold
    if (angularSpeed < 0.01) {
        angularSpeed = 0.0;
    } else {
        // Update rotation
        rotation = QQuaternion::fromAxisAndAngle(rotationAxis, angularSpeed) * rotation;

        // Request an update
        update();
    }
}
//! [1]

void MainWidget::initializeGL()
{
    initializeOpenGLFunctions();

    glClearColor(0, 0, 0, 1);

    initShaders();
    initCubeGeometry();

//! [2]
    // Enable depth buffer
    glEnable(GL_DEPTH_TEST);

    // Enable back face culling
    glEnable(GL_CULL_FACE);
//! [2]

    // Use QBasicTimer because its faster than QTimer
    timer.start(12, this);
}

//! [3]
void MainWidget::initShaders()
{
    // Compile vertex shader
    if (!program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/vshader.glsl"))
        close();

    // Compile fragment shader
    if (!program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/fshader.glsl"))
        close();

    // Link shader pipeline
    if (!program.link())
        close();

    // Bind shader pipeline for use
    if (!program.bind())
        close();
}
//! [3]

template <typename T, size_t N>
constexpr size_t countof(const T (&)[N]) { return N; }

void MainWidget::initCubeGeometry()
{
    arrayBuf = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
    indexBuf = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);

    arrayBuf->create();
    indexBuf->create();

    QVector3D vertices[] = {
        // Vertex data for face 0
        QVector3D(-1.0f, -1.0f,  1.0f), // v0
        QVector3D( 1.0f, -1.0f,  1.0f), // v1
        QVector3D(-1.0f,  1.0f,  1.0f), // v2
        QVector3D( 1.0f,  1.0f,  1.0f), // v3

        // Vertex data for face 1
        QVector3D( 1.0f, -1.0f,  1.0f), // v4
        QVector3D( 1.0f, -1.0f, -1.0f), // v5
        QVector3D( 1.0f,  1.0f,  1.0f), // v6
        QVector3D( 1.0f,  1.0f, -1.0f), // v7

        // Vertex data for face 2
        QVector3D( 1.0f, -1.0f, -1.0f), // v8
        QVector3D(-1.0f, -1.0f, -1.0f), // v9
        QVector3D( 1.0f,  1.0f, -1.0f), // v10
        QVector3D(-1.0f,  1.0f, -1.0f), // v11

        // Vertex data for face 3
        QVector3D(-1.0f, -1.0f, -1.0f), // v12
        QVector3D(-1.0f, -1.0f,  1.0f), // v13
        QVector3D(-1.0f,  1.0f, -1.0f), // v14
        QVector3D(-1.0f,  1.0f,  1.0f), // v15

        // Vertex data for face 4
        QVector3D(-1.0f, -1.0f, -1.0f), // v16
        QVector3D( 1.0f, -1.0f, -1.0f), // v17
        QVector3D(-1.0f, -1.0f,  1.0f), // v18
        QVector3D( 1.0f, -1.0f,  1.0f), // v19

        // Vertex data for face 5
        QVector3D(-1.0f,  1.0f,  1.0f), // v20
        QVector3D( 1.0f,  1.0f,  1.0f), // v21
        QVector3D(-1.0f,  1.0f, -1.0f), // v22
        QVector3D( 1.0f,  1.0f, -1.0f)  // v23
    };

    // Indices for drawing cube faces using triangles.
    GLushort indices[] = {
         0,  1,  2,  2,  1,  3,
         4,  5,  6,  6,  5,  7,
         8,  9, 10, 10,  9, 11,
        12, 13, 14, 14, 13, 15,
        16, 17, 18, 18, 17, 19,
        20, 21, 22, 22, 21, 23
    };

//! [1]
    // Transfer vertex data to VBO 0
    arrayBuf->bind();
    arrayBuf->allocate(vertices, countof(vertices) * sizeof(vertices[0]));

    // Transfer index data to VBO 1
    indexBuf->bind();
    indexBuf->allocate(indices, countof(indices) * sizeof(indices[0]));
//! [1]

    geometryElementCount = countof(indices);
}

//! [5]
void MainWidget::resizeGL(int w, int h)
{
    // Calculate aspect ratio
    qreal aspect = qreal(w) / qreal(h ? h : 1);

    // Set near plane to 3.0, far plane to 7.0, field of view 45 degrees
    const qreal zNear = 0.01, zFar = 100.0, fov = 45.0;

    // Reset projection
    projection.setToIdentity();

    // Set perspective projection
    projection.perspective(fov, aspect, zNear, zFar);
}
//! [5]

void MainWidget::paintGL()
{
    // Clear color and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glDisable(GL_DEPTH_TEST);
//! [6]
    // Calculate model view transformation
    QMatrix4x4 modelToWorldMatrix;
    modelToWorldMatrix.rotate(rotation);

    QMatrix4x4 cameraMatrix;
    cameraMatrix.translate(0.0, 0.0, -5.0);

    // Set modelview-projection matrix
    program.setUniformValue("mvp_matrix", projection * cameraMatrix * modelToWorldMatrix);
//! [6]


    // Draw cube geometry

    // Tell OpenGL which VBOs to use
    arrayBuf->bind();
    indexBuf->bind();

    // Tell OpenGL programmable pipeline how to locate vertex position data
    int vertexLocation = program.attributeLocation("a_position");
    program.enableAttributeArray(vertexLocation);
    program.setAttributeBuffer(vertexLocation, GL_FLOAT, /*offset=*/0, /*tupleSize=*/3, sizeof(QVector3D));

    glDrawElements(GL_TRIANGLES, geometryElementCount, GL_UNSIGNED_SHORT, nullptr);
}
