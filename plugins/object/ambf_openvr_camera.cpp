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

    \author    <amunawa2@jh.edu>
    \author    Adnan Munawar
*/
//==============================================================================

#include "ambf_openvr_camera.h"

using namespace std;


afOpenVRCamera::afOpenVRCamera()
{

}

int afOpenVRCamera::init(const afBaseObjectPtr a_afObjectPtr, const afBaseObjectAttribsPtr a_objectAttribs)
{

    m_camera = (afCameraPtr)a_afObjectPtr;
    m_ovrIfc = new OpenVRInterface();

    if (!m_ovrIfc->init(m_camera->getInternalCamera()->getNearClippingPlane(), m_camera->getInternalCamera()->getFarClippingPlane())){
        m_ovrIfc->close();
        return -1;
    }

    unsigned int rWidth, rHeight;
    m_ovrIfc->getRecommendedTextureSize(rWidth, rHeight);

    m_eyeRendererLeft = new EyeRenderer();
    m_eyeRendererLeft->init(m_camera, rWidth, rHeight);
    m_eyeRendererLeft->setProjectionMatrix(m_ovrIfc->getHMDProjectionMatrix(vr::Hmd_Eye::Eye_Left));
    m_eyeRendererLeft->setEyeInHMDTransform(m_ovrIfc->getEyePoseInHMD(vr::Hmd_Eye::Eye_Left));

    m_eyeRendererRight = new EyeRenderer();
    m_eyeRendererRight->init(m_camera, rWidth, rHeight);
    m_eyeRendererRight->setProjectionMatrix(m_ovrIfc->getHMDProjectionMatrix(vr::Hmd_Eye::Eye_Right));
    m_eyeRendererRight->setEyeInHMDTransform(m_ovrIfc->getEyePoseInHMD(vr::Hmd_Eye::Eye_Right));
    return true;
}

void afOpenVRCamera::graphicsUpdate(){
    m_ovrIfc->update();

    cTransform backupProjection = m_camera->getInternalCamera()->m_projectionMatrix;
    cTransform T_hmd = m_ovrIfc->getHMDPose();
//    cerr << "INFO! HMD Position " << T_hmd.getLocalPos().str() << endl;
    m_eyeRendererLeft->render(T_hmd);
    m_eyeRendererRight->render(T_hmd);
    m_ovrIfc->submitTextures(m_eyeRendererLeft->getTextureId(), m_eyeRendererRight->getTextureId());
    m_camera->getInternalCamera()->m_projectionMatrix = backupProjection;
}

void afOpenVRCamera::physicsUpdate(double dt){

}

void afOpenVRCamera::reset(){

}

bool afOpenVRCamera::close(){
    m_ovrIfc->close();
    delete m_ovrIfc;

    m_eyeRendererLeft->close();
    delete m_eyeRendererLeft;

    m_eyeRendererRight->close();
    delete m_eyeRendererRight;
    return true;
}


// OpenVR Related Stuff
bool OpenVRInterface::init(double nearPlane, double farPlane){
    vr::EVRInitError vrInitError = vr::VRInitError_None;
    m_vrSystem = vr::VR_Init(&vrInitError, vr::VRApplication_Scene);

    if (vrInitError != vr::VRInitError_None) {
        std::cerr << "ERROR! Failed to initialize OpenVR: " << vr::VR_GetVRInitErrorAsEnglishDescription(vrInitError) << std::endl;
        return false;
    }

    m_vrCompositor = vr::VRCompositor();
    if (!m_vrCompositor) {
        std::cerr << "ERROR! Failed to get VR Compositor" << std::endl;
        return false;
    }

    unsigned int rWidth, rHeight;
    m_vrSystem->GetRecommendedRenderTargetSize(&rWidth, &rHeight);

    std::cout << "INFO! Recommended Target Texture Size: " << rWidth << " x " <<  rHeight << std::endl;

    cMatrix3d rotOffset;
    rotOffset.setExtrinsicEulerRotationDeg(-90., -90., 0., C_EULER_ORDER_XYZ);
    m_ovrToAMBFOffset.setLocalRot(rotOffset);
    m_ovrToAMBFOffsetInv = m_ovrToAMBFOffset; m_ovrToAMBFOffsetInv.invert();

    m_near = nearPlane;
    m_far = farPlane;

    return true;
}

void OpenVRInterface::close(){
    if (m_vrSystem) {
        vr::VR_Shutdown();
        m_vrSystem = nullptr;
    }
}

void OpenVRInterface::update(){
    vr::VRCompositor()->WaitGetPoses(m_trackedDevicePoses, vr::k_unMaxTrackedDeviceCount, NULL, 0 );
}

void OpenVRInterface::submitTextures(GLuint leftTextureID, GLuint rightTextureID){
    // Define texture info to be submitted to OpenVR
    vr::Texture_t leftEyeTexture = { (void*)(uintptr_t)leftTextureID, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
    vr::Texture_t rightEyeTexture = { (void*)(uintptr_t)rightTextureID, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };

    // Submit to left and right eyes
    m_vrCompositor->Submit(vr::Eye_Left, &leftEyeTexture);
    m_vrCompositor->Submit(vr::Eye_Right, &rightEyeTexture);
}

void OpenVRInterface::getRecommendedTextureSize(unsigned int &width, unsigned int &height){
    m_vrSystem->GetRecommendedRenderTargetSize(&width, &height);
}

cTransform OpenVRInterface::getHMDPose(){
    cTransform trans;
    if (m_trackedDevicePoses[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid )
    {
        vr::HmdMatrix34_t pose = m_trackedDevicePoses[vr::k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking;
        vrMatrix_to_cTransform(pose, trans);
        trans = m_ovrToAMBFOffsetInv * trans * m_ovrToAMBFOffset;
    }
    return trans;
}

cTransform OpenVRInterface::getHMDProjectionMatrix(vr::Hmd_Eye nEye){
    cTransform trans;
    if ( !m_vrSystem )
        return trans;

    vr::HmdMatrix44_t mat = m_vrSystem->GetProjectionMatrix( nEye, m_near, m_far);
    vrMatrix_to_cTransform(mat, trans);
    return trans;
}


cTransform OpenVRInterface::getEyePoseInHMD(vr::Hmd_Eye nEye){
    cTransform trans;
    if ( !m_vrSystem )
        return trans;

    vr::HmdMatrix34_t eyeTrans = m_vrSystem->GetEyeToHeadTransform(nEye);
    vrMatrix_to_cTransform(eyeTrans, trans);
    trans.invert();

    return trans;
}


void OpenVRInterface::vrMatrix_to_cTransform(vr::HmdMatrix34_t& hmdMat, cTransform &trans){
    trans.set(hmdMat.m[0][0], hmdMat.m[1][0], hmdMat.m[2][0], 0.,
              hmdMat.m[0][1], hmdMat.m[1][1], hmdMat.m[2][1], 0.,
              hmdMat.m[0][2], hmdMat.m[1][2], hmdMat.m[2][2], 0.,
              hmdMat.m[0][3], hmdMat.m[1][3], hmdMat.m[2][3], 1.);
}

void OpenVRInterface::vrMatrix_to_cTransform(vr::HmdMatrix33_t& hmdMat, cTransform &trans){
    trans.set(hmdMat.m[0][0], hmdMat.m[1][0], hmdMat.m[2][0], 0.,
              hmdMat.m[0][1], hmdMat.m[1][1], hmdMat.m[2][1], 0.,
              hmdMat.m[0][2], hmdMat.m[1][2], hmdMat.m[2][2], 0.,
              0.            , 0.            , 0.            , 1.);
}

void OpenVRInterface::vrMatrix_to_cTransform(vr::HmdMatrix44_t& hmdMat, cTransform &trans){
    trans.set(hmdMat.m[0][0], hmdMat.m[1][0], hmdMat.m[2][0], hmdMat.m[3][0],
              hmdMat.m[0][1], hmdMat.m[1][1], hmdMat.m[2][1], hmdMat.m[3][1],
              hmdMat.m[0][2], hmdMat.m[1][2], hmdMat.m[2][2], hmdMat.m[3][2],
              hmdMat.m[0][3], hmdMat.m[1][3], hmdMat.m[2][3], hmdMat.m[3][3]);
}

void EyeRenderer::init(const afCameraPtr cam, double width, double height){
    m_width = width;
    m_height = height;
    m_camera = cam;
    m_frameBuffer = cFrameBuffer::create();
    m_frameBuffer->setup(m_camera->getInternalCamera(), m_width, m_height, true, true, GL_RGBA);
}


// Eye Rendered Implementation
void EyeRenderer::close(){
}

void EyeRenderer::render(cTransform& viewMat){
//    cTransform curProjection = m_camera->getInternalCamera()->m_projectionMatrix;
//    cTransform curTranform = m_camera->getLocalTransform();

    m_camera->getInternalCamera()->m_useCustomProjectionMatrix = true;
    m_camera->getInternalCamera()->m_projectionMatrix = m_projectMatrix;

    m_camera->setLocalTransform(viewMat * m_T_eye_hmd);
    m_frameBuffer->renderView();

//    m_camera->getInternalCamera()->m_useCustomProjectionMatrix = false;
//    m_camera->getInternalCamera()->m_projectionMatrix = curProjection;
//    m_camera->setLocalTransform(curTranform);
}

GLuint EyeRenderer::getTextureId(){
    return m_frameBuffer->m_imageBuffer->getTextureId();
}

void EyeRenderer::setProjectionMatrix(cTransform proj){
    m_projectMatrix = proj;
}

void EyeRenderer::setEyeInHMDTransform(cTransform trans){
    m_T_eye_hmd = trans;
}
