//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2024, AMBF
    (https://github.com/WPI-AIM/ambf)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <amunawar@jhu.edu>
    \author    Adnan Munawar
*/
//==============================================================================

// To silence warnings on MacOS
#define GL_SILENCE_DEPRECATION
#include <afFramework.h>
#include <stdio.h>
#include <openvr.h>

using namespace std;
using namespace ambf;


///
/// \brief The OpenVRInterface class
///
class OpenVRInterface{
public:
    OpenVRInterface(){}
    bool init(double nearPlane, double farPlane);
    void close();
    void update();
    void submitTextures(GLuint leftTextureID, GLuint rightTextureID);
    void getRecommendedTextureSize(unsigned int& width, unsigned int& height);

    cTransform getHMDPose();
    cTransform getHMDProjectionMatrix(vr::Hmd_Eye nEye);
    cTransform getEyePoseInHMD(vr::Hmd_Eye nEye);

protected:
    vr::IVRSystem* m_vrSystem = nullptr;
    vr::IVRCompositor* m_vrCompositor = nullptr;
    vr::TrackedDevicePose_t m_trackedDevicePoses[ vr::k_unMaxTrackedDeviceCount ];

    void vrMatrix_to_cTransform(vr::HmdMatrix33_t& hmdMat, cTransform& trans);
    void vrMatrix_to_cTransform(vr::HmdMatrix34_t& hmdMat, cTransform& trans);
    void vrMatrix_to_cTransform(vr::HmdMatrix44_t& hmdMat, cTransform& trans);

    cTransform m_ovrToAMBFOffset;
    cTransform m_ovrToAMBFOffsetInv;

    double m_near;
    double m_far;
};


///
/// \brief The EyeRenderer class
///
class EyeRenderer{
public:
    EyeRenderer(){}
    void init(const afCameraPtr cam, double width, double height);
    void close();

    void render(cTransform& viewMat);
    unsigned int getTextureId();
    void setProjectionMatrix(cTransform proj);
    void setEyeInHMDTransform(cTransform trans);

protected:
    cFrameBufferPtr m_frameBuffer;
    cTransform m_projectMatrix;
    cTransform m_T_eye_hmd;
    int m_width;
    int m_height;

    afCameraPtr m_camera;
};


///
/// \brief The afOpenVRCamera class
///
class afOpenVRCamera: public afObjectPlugin{
public:
    afOpenVRCamera();
    virtual int init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs) override;
    virtual void graphicsUpdate() override;
    virtual void physicsUpdate(double dt) override;
    virtual void reset() override;
    virtual bool close() override;

protected:
    afCameraPtr m_camera;
    EyeRenderer* m_eyeRendererLeft;
    EyeRenderer* m_eyeRendererRight;
    OpenVRInterface* m_ovrIfc;
};


AF_REGISTER_OBJECT_PLUGIN(afOpenVRCamera)
