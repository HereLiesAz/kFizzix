/*
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 	* Redistributions of source code must retain the above copyright notice,
 * 	  this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright notice,
 * 	  this list of conditions and the following disclaimer in the documentation
 * 	  and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
package com.hereliesaz.kfizzix.pooling;

import com.hereliesaz.kfizzix.collision.AABB;
import com.hereliesaz.kfizzix.collision.Collision;
import com.hereliesaz.kfizzix.collision.Distance;
import com.hereliesaz.kfizzix.collision.TimeOfImpact;
import com.hereliesaz.kfizzix.common.Mat22;
import com.hereliesaz.kfizzix.common.Mat33;
import com.hereliesaz.kfizzix.common.Rot;
import com.hereliesaz.kfizzix.common.Vec2;
import com.hereliesaz.kfizzix.common.Vec3;
import com.hereliesaz.kfizzix.dynamics.contacts.Contact;

/**
 * World pool interface
 *
 * @author Daniel Murphy
 */
public interface WorldPool
{
    DynamicStack<Contact> getPolyContactStack();

    DynamicStack<Contact> getCircleContactStack();

    DynamicStack<Contact> getPolyCircleContactStack();

    DynamicStack<Contact> getEdgeCircleContactStack();

    DynamicStack<Contact> getEdgePolyContactStack();

    DynamicStack<Contact> getChainCircleContactStack();

    DynamicStack<Contact> getChainPolyContactStack();

    Vec2 popVec2();

    Vec2[] popVec2(int num);

    void pushVec2(int num);

    Vec3 popVec3();

    Vec3[] popVec3(int num);

    void pushVec3(int num);

    Mat22 popMat22();

    Mat22[] popMat22(int num);

    void pushMat22(int num);

    Mat33 popMat33();

    void pushMat33(int num);

    AABB popAABB();

    AABB[] popAABB(int num);

    void pushAABB(int num);

    Rot popRot();

    void pushRot(int num);

    Collision getCollision();

    TimeOfImpact getTimeOfImpact();

    Distance getDistance();

    float[] getFloatArray(int argLength);

    int[] getIntArray(int argLength);

    Vec2[] getVec2Array(int argLength);
}
