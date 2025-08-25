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
 * ARISING IN ANY WAY OUT OF THE USE OF this SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
package com.hereliesaz.kfizzix.collision

import com.hereliesaz.kfizzix.collision.Distance.SimplexCache
import com.hereliesaz.kfizzix.collision.Manifold.ManifoldType
import com.hereliesaz.kfizzix.collision.shapes.CircleShape
import com.hereliesaz.kfizzix.collision.shapes.EdgeShape
import com.hereliesaz.kfizzix.collision.shapes.PolygonShape
import com.hereliesaz.kfizzix.collision.shapes.Shape
import com.hereliesaz.kfizzix.common.MathUtils
import com.hereliesaz.kfizzix.common.Rot
import com.hereliesaz.kfizzix.common.Settings
import com.hereliesaz.kfizzix.common.Transform
import com.hereliesaz.kfizzix.common.Vec2
import com.hereliesaz.kfizzix.pooling.WorldPool

/**
 * Functions used for computing contact points, distance queries, and TOI
 * queries. Collision methods are non-static for pooling speed, retrieve a
 * collision object from the {@code SingletonPool}. Should not be finalructed.
 *
 * @author Daniel Murphy
 */
class Collision(private val pool: WorldPool) {
    private val input = DistanceInput()
    private val cache = SimplexCache()
    private val output = DistanceOutput()

    /**
     * Determine if two generic shapes overlap.
     */
    fun testOverlap(
        shapeA: Shape, indexA: Int, shapeB: Shape,
        indexB: Int, xfA: Transform, xfB: Transform
    ): Boolean {
        input.proxyA.set(shapeA, indexA)
        input.proxyB.set(shapeB, indexB)
        input.transformA.set(xfA)
        input.transformB.set(xfB)
        input.useRadii = true
        cache.count = 0
        pool.distance.distance(output, cache, input)
        // djm note: anything significant about 10.0f?
        return output.distance < 10.0f * Settings.EPSILON
    }
    // #### COLLISION STUFF (not from collision.h or collision.cpp) ####
    /**
     * Compute the collision manifold between two circles.
     */
    fun collideCircles(
        manifold: Manifold, circle1: CircleShape, xfA: Transform,
        circle2: CircleShape, xfB: Transform
    ) {
        manifold.pointCount = 0
        // before inline:
        // Transform.mulToOut(xfA, circle1.p, pA);
        // Transform.mulToOut(xfB, circle2.p, pB);
        // d.set(pB).subLocal(pA);
        // float distSqr = d.x * d.x + d.y * d.y;
        // after inline:
        val circle1p = circle1.p
        val circle2p = circle2.p
        val pAx = (xfA.q.c * circle1p.x - xfA.q.s * circle1p.y) + xfA.p.x
        val pAy = (xfA.q.s * circle1p.x + xfA.q.c * circle1p.y) + xfA.p.y
        val pBx = (xfB.q.c * circle2p.x - xfB.q.s * circle2p.y) + xfB.p.x
        val pBy = (xfB.q.s * circle2p.x + xfB.q.c * circle2p.y) + xfB.p.y
        val dx = pBx - pAx
        val dy = pBy - pAy
        val distSqr = dx * dx + dy * dy
        // end inline
        val radius = circle1.radius + circle2.radius
        if (distSqr > radius * radius) {
            return
        }
        manifold.type = ManifoldType.CIRCLES
        manifold.localPoint.set(circle1p)
        manifold.localNormal.setZero()
        manifold.pointCount = 1
        manifold.points[0].localPoint.set(circle2p)
        manifold.points[0].id.zero()
    }
    // djm pooling, and from above
    /**
     * Compute the collision manifold between a polygon and a circle.
     */
    fun collidePolygonAndCircle(
        manifold: Manifold, polygon: PolygonShape, xfA: Transform,
        circle: CircleShape, xfB: Transform
    ) {
        manifold.pointCount = 0
        // Vec2 v = circle.p;
        // Compute circle position in the frame of the polygon.
        // before inline:
        // Transform.mulToOutUnsafe(xfB, circle.p, c);
        // Transform.mulTransToOut(xfA, c, cLocal);
        // final float cLocalX = cLocal.x;
        // final float cLocalY = cLocal.y;
        // after inline:
        val circleP = circle.p
        val xfBq = xfB.q
        val xfAq = xfA.q
        val cx = (xfBq.c * circleP.x - xfBq.s * circleP.y) + xfB.p.x
        val cy = (xfBq.s * circleP.x + xfBq.c * circleP.y) + xfB.p.y
        val px = cx - xfA.p.x
        val py = cy - xfA.p.y
        val cLocalX = (xfAq.c * px + xfAq.s * py)
        val cLocalY = (-xfAq.s * px + xfAq.c * py)
        // end inline
        // Find the min separating edge.
        var normalIndex = 0
        var separation = -Float.MAX_VALUE
        val radius = polygon.radius + circle.radius
        val vertexCount = polygon.count
        var s: Float
        val vertices = polygon.vertices
        val normals = polygon.normals
        for (i in 0 until vertexCount) {
            // before inline
            // temp.set(cLocal).subLocal(vertices[i]);
            // float s = Vec2.dot(normals[i], temp);
            // after inline
            val vertex = vertices[i]
            val tempX = cLocalX - vertex.x
            val tempY = cLocalY - vertex.y
            s = normals[i].x * tempX + normals[i].y * tempY
            if (s > radius) {
                // early out
                return
            }
            if (s > separation) {
                separation = s
                normalIndex = i
            }
        }
        // Vertices that subtend the incident face.
        val vertIndex1 = normalIndex
        val vertIndex2 = if (vertIndex1 + 1 < vertexCount) vertIndex1 + 1 else 0
        val v1 = vertices[vertIndex1]
        val v2 = vertices[vertIndex2]
        // If the center is inside the polygon ...
        if (separation < Settings.EPSILON) {
            manifold.pointCount = 1
            manifold.type = ManifoldType.FACE_A
            // before inline:
            // manifold.localNormal.set(normals[normalIndex]);
            // manifold.localPoint.set(v1).addLocal(v2).mulLocal(.5f);
            // manifold.points[0].localPoint.set(circle.p);
            // after inline:
            val normal = normals[normalIndex]
            manifold.localNormal.x = normal.x
            manifold.localNormal.y = normal.y
            manifold.localPoint.x = (v1.x + v2.x) * .5f
            manifold.localPoint.y = (v1.y + v2.y) * .5f
            val mPoint = manifold.points[0]
            mPoint.localPoint.x = circleP.x
            mPoint.localPoint.y = circleP.y
            mPoint.id.zero()
            // end inline
            return
        }
        // Compute barycentric coordinates
        // before inline:
        // temp.set(cLocal).subLocal(v1);
        // temp2.set(v2).subLocal(v1);
        // float u1 = Vec2.dot(temp, temp2);
        // temp.set(cLocal).subLocal(v2);
        // temp2.set(v1).subLocal(v2);
        // float u2 = Vec2.dot(temp, temp2);
        // after inline:
        val tempX = cLocalX - v1.x
        val tempY = cLocalY - v1.y
        val temp2X = v2.x - v1.x
        val temp2Y = v2.y - v1.y
        val u1 = tempX * temp2X + tempY * temp2Y
        val temp3X = cLocalX - v2.x
        val temp3Y = cLocalY - v2.y
        val temp4X = v1.x - v2.x
        val temp4Y = v1.y - v2.y
        val u2 = temp3X * temp4X + temp3Y * temp4Y
        // end inline
        if (u1 <= 0f) {
            // inlined
            val dx = cLocalX - v1.x
            val dy = cLocalY - v1.y
            if (dx * dx + dy * dy > radius * radius) {
                return
            }
            manifold.pointCount = 1
            manifold.type = ManifoldType.FACE_A
            // before inline:
            // manifold.localNormal.set(cLocal).subLocal(v1);
            // after inline:
            manifold.localNormal.x = cLocalX - v1.x
            manifold.localNormal.y = cLocalY - v1.y
            // end inline
            manifold.localNormal.normalize()
            manifold.localPoint.set(v1)
            manifold.points[0].localPoint.set(circleP)
            manifold.points[0].id.zero()
        } else if (u2 <= 0.0f) {
            // inlined
            val dx = cLocalX - v2.x
            val dy = cLocalY - v2.y
            if (dx * dx + dy * dy > radius * radius) {
                return
            }
            manifold.pointCount = 1
            manifold.type = ManifoldType.FACE_A
            // before inline:
            // manifold.localNormal.set(cLocal).subLocal(v2);
            // after inline:
            manifold.localNormal.x = cLocalX - v2.x
            manifold.localNormal.y = cLocalY - v2.y
            // end inline
            manifold.localNormal.normalize()
            manifold.localPoint.set(v2)
            manifold.points[0].localPoint.set(circleP)
            manifold.points[0].id.zero()
        } else {
            // Vec2 faceCenter = 0.5f * (v1 + v2);
            // (temp is faceCenter)
            // before inline:
            // temp.set(v1).addLocal(v2).mulLocal(.5f);
            //
            // temp2.set(cLocal).subLocal(temp);
            // separation = Vec2.dot(temp2, normals[vertIndex1]);
            // if (separation > radius) {
            // return;
            // }
            // after inline:
            val fcx = (v1.x + v2.x) * .5f
            val fcy = (v1.y + v2.y) * .5f
            val tx = cLocalX - fcx
            val ty = cLocalY - fcy
            val normal = normals[vertIndex1]
            separation = tx * normal.x + ty * normal.y
            if (separation > radius) {
                return
            }
            // end inline
            manifold.pointCount = 1
            manifold.type = ManifoldType.FACE_A
            manifold.localNormal.set(normals[vertIndex1])
            manifold.localPoint.x = fcx // (faceCenter)
            manifold.localPoint.y = fcy
            manifold.points[0].localPoint.set(circleP)
            manifold.points[0].id.zero()
        }
    }

    // djm pooling, and from above
    private val temp = Vec2()
    private val xf = Transform()
    private val n = Vec2()
    private val v1 = Vec2()

    /**
     * Find the max separation between poly1 and poly2 using edge normals from
     * poly1.
     */
    fun findMaxSeparation(
        results: EdgeResults, poly1: PolygonShape, xf1: Transform,
        poly2: PolygonShape, xf2: Transform
    ) {
        val count1 = poly1.count
        val count2 = poly2.count
        val n1s = poly1.normals
        val v1s = poly1.vertices
        val v2s = poly2.vertices
        Transform.mulTransToOutUnsafe(xf2, xf1, xf)
        val xfq = xf.q
        var bestIndex = 0
        var maxSeparation = -Float.MAX_VALUE
        for (i in 0 until count1) {
            // Get poly1 normal in frame2.
            Rot.mulToOutUnsafe(xfq, n1s[i], n)
            Transform.mulToOutUnsafe(xf, v1s[i], v1)
            // Find the deepest point for normal i.
            var si = Float.MAX_VALUE
            for (j in 0 until count2) {
                val v2sj = v2s[j]
                val sij = n.x * (v2sj.x - v1.x) + n.y * (v2sj.y - v1.y)
                if (sij < si) {
                    si = sij
                }
            }
            if (si > maxSeparation) {
                maxSeparation = si
                bestIndex = i
            }
        }
        results.edgeIndex = bestIndex
        results.separation = maxSeparation
    }

    fun findIncidentEdge(
        c: Array<ClipVertex>, poly1: PolygonShape, xf1: Transform, edge1: Int,
        poly2: PolygonShape, xf2: Transform
    ) {
        val count1 = poly1.count
        val normals1 = poly1.normals
        val count2 = poly2.count
        val vertices2 = poly2.vertices
        val normals2 = poly2.normals
        assert(0 <= edge1 && edge1 < count1)
        val c0 = c[0]
        val c1 = c[1]
        val xf1q = xf1.q
        val xf2q = xf2.q
        // Get the normal of the reference edge in poly2's frame.
        // Vec2 normal1 = MulT(xf2.R, Mul(xf1.R, normals1[edge1]));
        // before inline:
        // Rot.mulToOutUnsafe(xf1.q, normals1[edge1], normal1); // temporary
        // Rot.mulTrans(xf2.q, normal1, normal1);
        // after inline:
        val v = normals1[edge1]
        val tempX = xf1q.c * v.x - xf1q.s * v.y
        val tempY = xf1q.s * v.x + xf1q.c * v.y
        val normal1x = xf2q.c * tempX + xf2q.s * tempY
        val normal1y = -xf2q.s * tempX + xf2q.c * tempY
        // end inline
        // Find the incident edge on poly2.
        var index = 0
        var minDot = Float.MAX_VALUE
        for (i in 0 until count2) {
            val b = normals2[i]
            val dot = normal1x * b.x + normal1y * b.y
            if (dot < minDot) {
                minDot = dot
                index = i
            }
        }
        // Build the clip vertices for the incident edge.
        val i1 = index
        val i2 = if (i1 + 1 < count2) i1 + 1 else 0
        // c0.v = Mul(xf2, vertices2[i1]);
        val v1 = vertices2[i1]
        val out = c0.v
        out.x = (xf2q.c * v1.x - xf2q.s * v1.y) + xf2.p.x
        out.y = (xf2q.s * v1.x + xf2q.c * v1.y) + xf2.p.y
        c0.id.indexA = edge1.toByte()
        c0.id.indexB = i1.toByte()
        c0.id.typeA = ContactID.Type.FACE.ordinal.toByte()
        c0.id.typeB = ContactID.Type.VERTEX.ordinal.toByte()
        // c1.v = Mul(xf2, vertices2[i2]);
        val v2 = vertices2[i2]
        val out1 = c1.v
        out1.x = (xf2q.c * v2.x - xf2q.s * v2.y) + xf2.p.x
        out1.y = (xf2q.s * v2.x + xf2q.c * v2.y) + xf2.p.y
        c1.id.indexA = edge1.toByte()
        c1.id.indexB = i2.toByte()
        c1.id.typeA = ContactID.Type.FACE.ordinal.toByte()
        c1.id.typeB = ContactID.Type.VERTEX.ordinal.toByte()
    }

    private val results1 = EdgeResults()
    private val results2 = EdgeResults()
    private val incidentEdge = Array(2) { ClipVertex() }
    private val localTangent = Vec2()
    private val localNormal = Vec2()
    private val planePoint = Vec2()
    private val tangent = Vec2()
    private val v11 = Vec2()
    private val v12 = Vec2()
    private val clipPoints1 = Array(2) { ClipVertex() }
    private val clipPoints2 = Array(2) { ClipVertex() }

    /**
     * Compute the collision manifold between two polygons.
     */
    fun collidePolygons(
        manifold: Manifold, polyA: PolygonShape, xfA: Transform,
        polyB: PolygonShape, xfB: Transform
    ) {
        // Find edge normal of max separation on A - return if separating axis
        // is found
        // Find edge normal of max separation on B - return if separation axis
        // is found
        // Choose reference edge as min(minA, minB)
        // Find incident edge
        // Clip
        // The normal points from 1 to 2
        manifold.pointCount = 0
        val totalRadius = polyA.radius + polyB.radius
        findMaxSeparation(results1, polyA, xfA, polyB, xfB)
        if (results1.separation > totalRadius) {
            return
        }
        findMaxSeparation(results2, polyB, xfB, polyA, xfA)
        if (results2.separation > totalRadius) {
            return
        }
        val poly1: PolygonShape // reference polygon
        val poly2: PolygonShape // incident polygon
        val xf1: Transform
        val xf2: Transform
        val edge1: Int // reference edge
        val flip: Boolean
        val tol = 0.1f * Settings.linearSlop
        if (results2.separation > results1.separation + tol) {
            poly1 = polyB
            poly2 = polyA
            xf1 = xfB
            xf2 = xfA
            edge1 = results2.edgeIndex
            manifold.type = ManifoldType.FACE_B
            flip = true
        } else {
            poly1 = polyA
            poly2 = polyB
            xf1 = xfA
            xf2 = xfB
            edge1 = results1.edgeIndex
            manifold.type = ManifoldType.FACE_A
            flip = false
        }
        val xf1q = xf1.q
        findIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2)
        val count1 = poly1.count
        val vertices1 = poly1.vertices
        val iv1 = edge1
        val iv2 = if (edge1 + 1 < count1) edge1 + 1 else 0
        v11.set(vertices1[iv1])
        v12.set(vertices1[iv2])
        localTangent.x = v12.x - v11.x
        localTangent.y = v12.y - v11.y
        localTangent.normalize()
        // Vec2 localNormal = Vec2.cross(dv, 1.0f);
        localNormal.x = localTangent.y
        localNormal.y = -1f * localTangent.x
        // Vec2 planePoint = 0.5f * (v11+ v12);
        planePoint.x = (v11.x + v12.x) * .5f
        planePoint.y = (v11.y + v12.y) * .5f
        // Rot.mulToOutUnsafe(xf1.q, localTangent, tangent);
        tangent.x = xf1q.c * localTangent.x - xf1q.s * localTangent.y
        tangent.y = xf1q.s * localTangent.x + xf1q.c * localTangent.y
        // Vec2.crossToOutUnsafe(tangent, 1f, normal);
        val normalx = tangent.y
        val normaly = -1f * tangent.x
        Transform.mulToOut(xf1, v11, v11)
        Transform.mulToOut(xf1, v12, v12)
        // v11 = Mul(xf1, v11);
        // v12 = Mul(xf1, v12);
        // Face offset
        // float frontOffset = Vec2.dot(normal, v11);
        val frontOffset = normalx * v11.x + normaly * v11.y
        // Side offsets, extended by polytope skin thickness.
        // float sideOffset1 = -Vec2.dot(tangent, v11) + totalRadius;
        // float sideOffset2 = Vec2.dot(tangent, v12) + totalRadius;
        val sideOffset1 = -(tangent.x * v11.x + tangent.y * v11.y) + totalRadius
        val sideOffset2 = tangent.x * v12.x + tangent.y * v12.y + totalRadius
        // Clip incident edge against extruded edge1 side edges.
        // ClipVertex clipPoints1[2];
        // ClipVertex clipPoints2[2];
        var np: Int
        // Clip to box side 1
        // np = ClipSegmentToLine(clipPoints1, incidentEdge, -sideNormal,
        // sideOffset1);
        tangent.negateLocal()
        np = clipSegmentToLine(
            clipPoints1, incidentEdge, tangent, sideOffset1,
            iv1
        )
        tangent.negateLocal()
        if (np < 2) {
            return
        }
        // Clip to negative box side 1
        np = clipSegmentToLine(
            clipPoints2, clipPoints1, tangent, sideOffset2,
            iv2
        )
        if (np < 2) {
            return
        }
        // Now clipPoints2 contains the clipped points.
        manifold.localNormal.set(localNormal)
        manifold.localPoint.set(planePoint)
        var pointCount = 0
        for (i in 0 until Settings.maxManifoldPoints) {
            // float separation = Vec2.dot(normal, clipPoints2[i].v) -
            // frontOffset;
            val separation = normalx * clipPoints2[i].v.x + normaly * clipPoints2[i].v.y - frontOffset
            if (separation <= totalRadius) {
                val cp = manifold.points[pointCount]
                // cp.localPoint = MulT(xf2, clipPoints2[i].v);
                val out = cp.localPoint
                val px = clipPoints2[i].v.x - xf2.p.x
                val py = clipPoints2[i].v.y - xf2.p.y
                out.x = (xf2.q.c * px + xf2.q.s * py)
                out.y = (-xf2.q.s * px + xf2.q.c * py)
                cp.id.set(clipPoints2[i].id)
                if (flip) {
                    // Swap features
                    cp.id.flip()
                }
                ++pointCount
            }
        }
        manifold.pointCount = pointCount
    }

    private val Q = Vec2()
    private val e = Vec2()
    private val cf = ContactID()
    private val e1 = Vec2()
    private val P = Vec2()

    // Compute contact points for edge versus circle.
    // This accounts for edge connectivity.
    fun collideEdgeAndCircle(
        manifold: Manifold, edgeA: EdgeShape,
        xfA: Transform, circleB: CircleShape, xfB: Transform
    ) {
        manifold.pointCount = 0
        // Compute circle in frame of edge
        // Vec2 Q = MulT(xfA, Mul(xfB, circleB.p));
        Transform.mulToOutUnsafe(xfB, circleB.p, temp)
        Transform.mulTransToOutUnsafe(xfA, temp, Q)
        val A = edgeA.vertex1
        val B = edgeA.vertex2
        e.set(B).subLocal(A)
        // Barycentric coordinates
        val u = Vec2.dot(e, temp.set(B).subLocal(Q))
        val v = Vec2.dot(e, temp.set(Q).subLocal(A))
        val radius = edgeA.radius + circleB.radius
        // ContactFeature cf;
        cf.indexB = 0
        cf.typeB = ContactID.Type.VERTEX.ordinal.toByte()
        // Region A
        if (v <= 0.0f) {
            d.set(Q).subLocal(A)
            val dd = Vec2.dot(d, d)
            if (dd > radius * radius) {
                return
            }
            // Is there an edge connected to A?
            if (edgeA.hasVertex0) {
                val A1 = edgeA.vertex0
                e1.set(A).subLocal(A1)
                val u1 = Vec2.dot(e1, temp.set(A).subLocal(Q))
                // Is the circle in Region AB of the previous edge?
                if (u1 > 0.0f) {
                    return
                }
            }
            cf.indexA = 0
            cf.typeA = ContactID.Type.VERTEX.ordinal.toByte()
            manifold.pointCount = 1
            manifold.type = Manifold.ManifoldType.CIRCLES
            manifold.localNormal.setZero()
            manifold.localPoint.set(A)
            // manifold.points[0].id.key = 0;
            manifold.points[0].id.set(cf)
            manifold.points[0].localPoint.set(circleB.p)
            return
        }
        // Region B
        if (u <= 0.0f) {
            d.set(Q).subLocal(B)
            val dd = Vec2.dot(d, d)
            if (dd > radius * radius) {
                return
            }
            // Is there an edge connected to B?
            if (edgeA.hasVertex3) {
                val B2 = edgeA.vertex3
                val e2 = e1
                e2.set(B2).subLocal(B)
                val v2 = Vec2.dot(e2, temp.set(Q).subLocal(B))
                // Is the circle in Region AB of the next edge?
                if (v2 > 0.0f) {
                    return
                }
            }
            cf.indexA = 1
            cf.typeA = ContactID.Type.VERTEX.ordinal.toByte()
            manifold.pointCount = 1
            manifold.type = Manifold.ManifoldType.CIRCLES
            manifold.localNormal.setZero()
            manifold.localPoint.set(B)
            // manifold.points[0].id.key = 0;
            manifold.points[0].id.set(cf)
            manifold.points[0].localPoint.set(circleB.p)
            return
        }
        // Region AB
        val den = Vec2.dot(e, e)
        assert(den > 0.0f)
        // Vec2 P = (1.0f / den) * (u * A + v * B);
        P.set(A).mulLocal(u).addLocal(temp.set(B).mulLocal(v))
        P.mulLocal(1.0f / den)
        d.set(Q).subLocal(P)
        val dd = Vec2.dot(d, d)
        if (dd > radius * radius) {
            return
        }
        n.x = -e.y
        n.y = e.x
        if (Vec2.dot(n, temp.set(Q).subLocal(A)) < 0.0f) {
            n.set(-n.x, -n.y)
        }
        n.normalize()
        cf.indexA = 0
        cf.typeA = ContactID.Type.FACE.ordinal.toByte()
        manifold.pointCount = 1
        manifold.type = Manifold.ManifoldType.FACE_A
        manifold.localNormal.set(n)
        manifold.localPoint.set(A)
        // manifold.points[0].id.key = 0;
        manifold.points[0].id.set(cf)
        manifold.points[0].localPoint.set(circleB.p)
    }

    private val collider = EPCollider()
    fun collideEdgeAndPolygon(
        manifold: Manifold, edgeA: EdgeShape,
        xfA: Transform, polygonB: PolygonShape,
        xfB: Transform
    ) {
        collider.collide(manifold, edgeA, xfA, polygonB, xfB)
    }

    /**
     * Java-specific class for returning edge results
     */
    class EdgeResults {
        var separation = 0f
        var edgeIndex = 0
    }

    /**
     * Used for computing contact manifolds.
     */
    class ClipVertex {
        val v: Vec2
        val id: ContactID

        constructor() {
            v = Vec2()
            id = ContactID()
        }

        fun set(cv: ClipVertex) {
            val v1 = cv.v
            v.x = v1.x
            v.y = v1.y
            val c = cv.id
            id.indexA = c.indexA
            id.indexB = c.indexB
            id.typeA = c.typeA
            id.typeB = c.typeB
        }
    }

    /**
     * This is used for determining the state of contact points.
     *
     * @author Daniel Murphy
     */
    enum class PointState {
        /**
         * point does not exist
         */
        NULL_STATE,

        /**
         * point was added in the update
         */
        ADD_STATE,

        /**
         * point persisted across the update
         */
        PERSIST_STATE,

        /**
         * point was removed in the update
         */
        REMOVE_STATE
    }

    /**
     * This structure is used to keep track of the best separating axis.
     */
    internal class EPAxis {
        enum class Type {
            UNKNOWN, EDGE_A, EDGE_B
        }

        var type: Type? = null
        var index = 0
        var separation = 0f
    }

    /**
     * This holds polygon B expressed in frame A.
     */
    internal class TempPolygon {
        val vertices = Array(Settings.maxPolygonVertices) { Vec2() }
        val normals = Array(Settings.maxPolygonVertices) { Vec2() }
        var count = 0
    }

    /**
     * Reference face used for clipping
     */
    internal class ReferenceFace {
        var i1 = 0
        var i2 = 0
        val v1 = Vec2()
        val v2 = Vec2()
        val normal = Vec2()
        val sideNormal1 = Vec2()
        var sideOffset1 = 0f
        val sideNormal2 = Vec2()
        var sideOffset2 = 0f
    }

    /**
     * This class collides and edge and a polygon, taking into account edge
     * adjacency.
     */
    internal class EPCollider {
        enum class VertexType {
            ISOLATED, CONCAVE, CONVEX
        }

        val polygonB = TempPolygon()
        val xf = Transform()
        val centroidB = Vec2()
        var v0 = Vec2()
        var v1 = Vec2()
        var v2 = Vec2()
        var v3 = Vec2()
        val normal0 = Vec2()
        val normal1 = Vec2()
        val normal2 = Vec2()
        val normal = Vec2()
        var type1: VertexType? = null
        var type2: VertexType? = null
        val lowerLimit = Vec2()
        val upperLimit = Vec2()
        var radius = 0f
        var front = false
        private val edge1 = Vec2()
        private val temp = Vec2()
        private val edge0 = Vec2()
        private val edge2 = Vec2()
        private val ie = Array(2) { ClipVertex() }
        private val clipPoints1 = Array(2) { ClipVertex() }
        private val clipPoints2 = Array(2) { ClipVertex() }
        private val rf = ReferenceFace()
        private val edgeAxis = EPAxis()
        private val polygonAxis = EPAxis()
        fun collide(
            manifold: Manifold, edgeA: EdgeShape,
            xfA: Transform, polygonB: PolygonShape,
            xfB: Transform
        ) {
            Transform.mulTransToOutUnsafe(xfA, xfB, xf)
            Transform.mulToOutUnsafe(xf, polygonB.centroid, centroidB)
            v0 = edgeA.vertex0
            v1 = edgeA.vertex1
            v2 = edgeA.vertex2
            v3 = edgeA.vertex3
            val hasVertex0 = edgeA.hasVertex0
            val hasVertex3 = edgeA.hasVertex3
            edge1.set(v2).subLocal(v1)
            edge1.normalize()
            normal1.set(edge1.y, -edge1.x)
            val offset1 = Vec2.dot(normal1, temp.set(centroidB).subLocal(v1))
            var offset0 = 0.0f
            var offset2 = 0.0f
            var convex1 = false
            var convex2 = false
            // Is there a preceding edge?
            if (hasVertex0) {
                edge0.set(v1).subLocal(v0)
                edge0.normalize()
                normal0.set(edge0.y, -edge0.x)
                convex1 = Vec2.cross(edge0, edge1) >= 0.0f
                offset0 = Vec2.dot(normal0, temp.set(centroidB).subLocal(v0))
            }
            // Is there a following edge?
            if (hasVertex3) {
                edge2.set(v3).subLocal(v2)
                edge2.normalize()
                normal2.set(edge2.y, -edge2.x)
                convex2 = Vec2.cross(edge1, edge2) > 0.0f
                offset2 = Vec2.dot(normal2, temp.set(centroidB).subLocal(v2))
            }
            // Determine front or back collision. Determine collision normal
            // limits.
            if (hasVertex0 && hasVertex3) {
                if (convex1 && convex2) {
                    front = offset0 >= 0.0f || offset1 >= 0.0f || offset2 >= 0.0f
                    if (front) {
                        normal.x = normal1.x
                        normal.y = normal1.y
                        lowerLimit.x = normal0.x
                        lowerLimit.y = normal0.y
                        upperLimit.x = normal2.x
                        upperLimit.y = normal2.y
                    } else {
                        normal.x = -normal1.x
                        normal.y = -normal1.y
                        lowerLimit.x = -normal1.x
                        lowerLimit.y = -normal1.y
                        upperLimit.x = -normal1.x
                        upperLimit.y = -normal1.y
                    }
                } else if (convex1) {
                    front = offset0 >= 0.0f || offset1 >= 0.0f && offset2 >= 0.0f
                    if (front) {
                        normal.x = normal1.x
                        normal.y = normal1.y
                        lowerLimit.x = normal0.x
                        lowerLimit.y = normal0.y
                        upperLimit.x = normal1.x
                        upperLimit.y = normal1.y
                    } else {
                        normal.x = -normal1.x
                        normal.y = -normal1.y
                        lowerLimit.x = -normal2.x
                        lowerLimit.y = -normal2.y
                        upperLimit.x = -normal1.x
                        upperLimit.y = -normal1.y
                    }
                } else if (convex2) {
                    front = offset2 >= 0.0f || offset0 >= 0.0f && offset1 >= 0.0f
                    if (front) {
                        normal.x = normal1.x
                        normal.y = normal1.y
                        lowerLimit.x = normal1.x
                        lowerLimit.y = normal1.y
                        upperLimit.x = normal2.x
                        upperLimit.y = normal2.y
                    } else {
                        normal.x = -normal1.x
                        normal.y = -normal1.y
                        lowerLimit.x = -normal1.x
                        lowerLimit.y = -normal1.y
                        upperLimit.x = -normal0.x
                        upperLimit.y = -normal0.y
                    }
                } else {
                    front = offset0 >= 0.0f && offset1 >= 0.0f && offset2 >= 0.0f
                    if (front) {
                        normal.x = normal1.x
                        normal.y = normal1.y
                        lowerLimit.x = normal1.x
                        lowerLimit.y = normal1.y
                        upperLimit.x = normal1.x
                        upperLimit.y = normal1.y
                    } else {
                        normal.x = -normal1.x
                        normal.y = -normal1.y
                        lowerLimit.x = -normal2.x
                        lowerLimit.y = -normal2.y
                        upperLimit.x = -normal0.x
                        upperLimit.y = -normal0.y
                    }
                }
            } else if (hasVertex0) {
                if (convex1) {
                    front = offset0 >= 0.0f || offset1 >= 0.0f
                    if (front) {
                        normal.x = normal1.x
                        normal.y = normal1.y
                        lowerLimit.x = normal0.x
                        lowerLimit.y = normal0.y
                    } else {
                        normal.x = -normal1.x
                        normal.y = -normal1.y
                        lowerLimit.x = normal1.x
                        lowerLimit.y = normal1.y
                    }
                    upperLimit.x = -normal1.x
                    upperLimit.y = -normal1.y
                } else {
                    front = offset0 >= 0.0f && offset1 >= 0.0f
                    if (front) {
                        normal.x = normal1.x
                        normal.y = normal1.y
                        lowerLimit.x = normal1.x
                        lowerLimit.y = normal1.y
                        upperLimit.x = -normal1.x
                        upperLimit.y = -normal1.y
                    } else {
                        normal.x = -normal1.x
                        normal.y = -normal1.y
                        lowerLimit.x = normal1.x
                        lowerLimit.y = normal1.y
                        upperLimit.x = -normal0.x
                        upperLimit.y = -normal0.y
                    }
                }
            } else if (hasVertex3) {
                if (convex2) {
                    front = offset1 >= 0.0f || offset2 >= 0.0f
                    if (front) {
                        normal.x = normal1.x
                        normal.y = normal1.y
                        lowerLimit.x = -normal1.x
                        lowerLimit.y = -normal1.y
                        upperLimit.x = normal2.x
                        upperLimit.y = normal2.y
                    } else {
                        normal.x = -normal1.x
                        normal.y = -normal1.y
                        lowerLimit.x = -normal1.x
                        lowerLimit.y = -normal1.y
                        upperLimit.x = normal1.x
                        upperLimit.y = normal1.y
                    }
                } else {
                    front = offset1 >= 0.0f && offset2 >= 0.0f
                    if (front) {
                        normal.x = normal1.x
                        normal.y = normal1.y
                        lowerLimit.x = -normal1.x
                        lowerLimit.y = -normal1.y
                    } else {
                        normal.x = -normal1.x
                        normal.y = -normal1.y
                        lowerLimit.x = -normal2.x
                        lowerLimit.y = -normal2.y
                    }
                    upperLimit.x = normal1.x
                    upperLimit.y = normal1.y
                }
            } else {
                front = offset1 >= 0.0f
                if (front) {
                    normal.x = normal1.x
                    normal.y = normal1.y
                    lowerLimit.x = -normal1.x
                    lowerLimit.y = -normal1.y
                    upperLimit.x = -normal1.x
                    upperLimit.y = -normal1.y
                } else {
                    normal.x = -normal1.x
                    normal.y = -normal1.y
                    lowerLimit.x = normal1.x
                    lowerLimit.y = normal1.y
                    upperLimit.x = normal1.x
                    upperLimit.y = normal1.y
                }
            }
            // Get polygonB in frameA
            this.polygonB.count = polygonB.count
            for (i in 0 until polygonB.count) {
                Transform.mulToOutUnsafe(xf, polygonB.vertices[i], this.polygonB.vertices[i])
                Rot.mulToOutUnsafe(xf.q, polygonB.normals[i], this.polygonB.normals[i])
            }
            radius = 2.0f * Settings.polygonRadius
            manifold.pointCount = 0
            computeEdgeSeparation(edgeAxis)
            // If no valid normal can be found than this edge should not
            // collide.
            if (edgeAxis.type == EPAxis.Type.UNKNOWN) {
                return
            }
            if (edgeAxis.separation > radius) {
                return
            }
            computePolygonSeparation(polygonAxis)
            if (polygonAxis.type != EPAxis.Type.UNKNOWN && polygonAxis.separation > radius) {
                return
            }
            // Use hysteresis for jitter reduction.
            val primaryAxis = epAxis
            val ie0 = ie[0]
            val ie1 = ie[1]
            if (primaryAxis.type == EPAxis.Type.EDGE_A) {
                manifold.type = Manifold.ManifoldType.FACE_A
                // Search for the polygon normal that is the most antiparallel
                // to
                // the edge normal.
                val i1 = i1
                val i2 = if (i1 + 1 < this.polygonB.count) i1 + 1 else 0
                ie0.v.set(this.polygonB.vertices[i1])
                ie0.id.indexA = 0
                ie0.id.indexB = i1.toByte()
                ie0.id.typeA = ContactID.Type.FACE.ordinal.toByte()
                ie0.id.typeB = ContactID.Type.VERTEX.ordinal.toByte()
                ie1.v.set(this.polygonB.vertices[i2])
                ie1.id.indexA = 0
                ie1.id.indexB = i2.toByte()
                ie1.id.typeA = ContactID.Type.FACE.ordinal.toByte()
                ie1.id.typeB = ContactID.Type.VERTEX.ordinal.toByte()
                if (front) {
                    rf.i1 = 0
                    rf.i2 = 1
                    rf.v1.set(v1)
                    rf.v2.set(v2)
                    rf.normal.set(normal1)
                } else {
                    rf.i1 = 1
                    rf.i2 = 0
                    rf.v1.set(v2)
                    rf.v2.set(v1)
                    rf.normal.set(normal1).negateLocal()
                }
            } else {
                manifold.type = Manifold.ManifoldType.FACE_B
                ie0.v.set(v1)
                ie0.id.indexA = 0
                ie0.id.indexB = primaryAxis.index.toByte()
                ie0.id.typeA = ContactID.Type.VERTEX.ordinal.toByte()
                ie0.id.typeB = ContactID.Type.FACE.ordinal.toByte()
                ie1.v.set(v2)
                ie1.id.indexA = 0
                ie1.id.indexB = primaryAxis.index.toByte()
                ie1.id.typeA = ContactID.Type.VERTEX.ordinal.toByte()
                ie1.id.typeB = ContactID.Type.FACE.ordinal.toByte()
                rf.i1 = primaryAxis.index
                rf.i2 = if (rf.i1 + 1 < this.polygonB.count) rf.i1 + 1 else 0
                rf.v1.set(this.polygonB.vertices[rf.i1])
                rf.v2.set(this.polygonB.vertices[rf.i2])
                rf.normal.set(this.polygonB.normals[rf.i1])
            }
            rf.sideNormal1.set(rf.normal.y, -rf.normal.x)
            rf.sideNormal2.set(rf.sideNormal1).negateLocal()
            rf.sideOffset1 = Vec2.dot(rf.sideNormal1, rf.v1)
            rf.sideOffset2 = Vec2.dot(rf.sideNormal2, rf.v2)
            // Clip incident edge against extruded edge1 side edges.
            var np: Int
            // Clip to box side 1
            np = clipSegmentToLine(
                clipPoints1, ie, rf.sideNormal1,
                rf.sideOffset1, rf.i1
            )
            if (np < Settings.maxManifoldPoints) {
                return
            }
            // Clip to negative box side 1
            np = clipSegmentToLine(
                clipPoints2, clipPoints1, rf.sideNormal2,
                rf.sideOffset2, rf.i2
            )
            if (np < Settings.maxManifoldPoints) {
                return
            }
            // Now clipPoints2 contains the clipped points.
            if (primaryAxis.type == EPAxis.Type.EDGE_A) {
                manifold.localNormal.set(rf.normal)
                manifold.localPoint.set(rf.v1)
            } else {
                manifold.localNormal.set(polygonB.normals[rf.i1])
                manifold.localPoint.set(polygonB.vertices[rf.i1])
            }
            var pointCount = 0
            for (i in 0 until Settings.maxManifoldPoints) {
                var separation: Float
                separation = Vec2.dot(
                    rf.normal,
                    temp.set(clipPoints2[i].v).subLocal(rf.v1)
                )
                if (separation <= radius) {
                    val cp = manifold.points[pointCount]
                    if (primaryAxis.type == EPAxis.Type.EDGE_A) {
                        // cp.localPoint = MulT(xf, clipPoints2[i].v);
                        Transform.mulTransToOutUnsafe(xf, clipPoints2[i].v, cp.localPoint)
                        cp.id.set(clipPoints2[i].id)
                    } else {
                        cp.localPoint.set(clipPoints2[i].v)
                        cp.id.typeA = clipPoints2[i].id.typeB
                        cp.id.typeB = clipPoints2[i].id.typeA
                        cp.id.indexA = clipPoints2[i].id.indexB
                        cp.id.indexB = clipPoints2[i].id.indexA
                    }
                    ++pointCount
                }
            }
            manifold.pointCount = pointCount
        }

        private val i1: Int
            private get() {
                var bestIndex = 0
                var bestValue = Vec2.dot(normal, polygonB.normals[0])
                for (i in 1 until polygonB.count) {
                    val value = Vec2.dot(normal, polygonB.normals[i])
                    if (value < bestValue) {
                        bestValue = value
                        bestIndex = i
                    }
                }
                return bestIndex
            }
        private val epAxis: EPAxis
            private get() {
                val relativeTol = 0.98f
                val absoluteTol = 0.001f
                val primaryAxis: EPAxis
                primaryAxis = if (polygonAxis.type == EPAxis.Type.UNKNOWN) {
                    edgeAxis
                } else if (polygonAxis.separation > relativeTol * edgeAxis.separation + absoluteTol) {
                    polygonAxis
                } else {
                    edgeAxis
                }
                return primaryAxis
            }

        fun computeEdgeSeparation(axis: EPAxis) {
            axis.type = EPAxis.Type.EDGE_A
            axis.index = if (front) 0 else 1
            axis.separation = Float.MAX_VALUE
            val nx = normal.x
            val ny = normal.y
            for (i in 0 until polygonB.count) {
                val v = polygonB.vertices[i]
                val tempX = v.x - v1.x
                val tempY = v.y - v1.y
                val s = nx * tempX + ny * tempY
                if (s < axis.separation) {
                    axis.separation = s
                }
            }
        }

        private val perp = Vec2()
        private val n2 = Vec2()
        fun computePolygonSeparation(axis: EPAxis) {
            axis.type = EPAxis.Type.UNKNOWN
            axis.index = -1
            axis.separation = -Float.MAX_VALUE
            perp.x = -normal.y
            perp.y = normal.x
            for (i in 0 until polygonB.count) {
                val normalB = polygonB.normals[i]
                val vB = polygonB.vertices[i]
                n2.x = -normalB.x
                n2.y = -normalB.y
                // float s1 = Vec2.dot(n, temp.set(vB).subLocal(v1));
                // float s2 = Vec2.dot(n, temp.set(vB).subLocal(v2));
                var tempX = vB.x - v1.x
                var tempY = vB.y - v1.y
                val s1 = n2.x * tempX + n2.y * tempY
                tempX = vB.x - v2.x
                tempY = vB.y - v2.y
                val s2 = n2.x * tempX + n2.y * tempY
                val s = MathUtils.min(s1, s2)
                if (s > radius) {
                    // No collision
                    axis.type = EPAxis.Type.EDGE_B
                    axis.index = i
                    axis.separation = s
                    return
                }
                // Adjacency
                if (n2.x * perp.x + n2.y * perp.y >= 0.0f) {
                    if (Vec2.dot(
                            temp.set(n2).subLocal(upperLimit),
                            normal
                        ) < -Settings.angularSlop
                    ) {
                        continue
                    }
                } else {
                    if (Vec2.dot(
                            temp.set(n2).subLocal(lowerLimit),
                            normal
                        ) < -Settings.angularSlop
                    ) {
                        continue
                    }
                }
                if (s > axis.separation) {
                    axis.type = EPAxis.Type.EDGE_B
                    axis.index = i
                    axis.separation = s
                }
            }
        }

    }

    companion object {
        const val NULL_FEATURE = Int.MAX_VALUE

        /**
         * Compute the point states given two manifolds. The states pertain to the
         * transition from manifold1 to manifold2. So state1 is either persist or
         * remove while state2 is either add or persist.
         */
        fun getPointStates(
            state1: Array<PointState>, state2: Array<PointState>,
            manifold1: Manifold, manifold2: Manifold
        ) {
            for (i in 0 until Settings.maxManifoldPoints) {
                state1[i] = PointState.NULL_STATE
                state2[i] = PointState.NULL_STATE
            }
            // Detect persists and removes.
            for (i in 0 until manifold1.pointCount) {
                val id = manifold1.points[i].id
                state1[i] = PointState.REMOVE_STATE
                for (j in 0 until manifold2.pointCount) {
                    if (manifold2.points[j].id.isEqual(id)) {
                        state1[i] = PointState.PERSIST_STATE
                        break
                    }
                }
            }
            // Detect persists and adds
            for (i in 0 until manifold2.pointCount) {
                val id = manifold2.points[i].id
                state2[i] = PointState.ADD_STATE
                for (j in 0 until manifold1.pointCount) {
                    if (manifold1.points[j].id.isEqual(id)) {
                        state2[i] = PointState.PERSIST_STATE
                        break
                    }
                }
            }
        }

        /**
         * Clipping for contact manifolds. Sutherland-Hodgman clipping.
         */
        fun clipSegmentToLine(
            vOut: Array<ClipVertex>, vIn: Array<ClipVertex>,
            normal: Vec2, offset: Float, vertexIndexA: Int
        ): Int {
            // Start with no output points
            var numOut = 0
            val vIn0 = vIn[0]
            val vIn1 = vIn[1]
            val vIn0v = vIn0.v
            val vIn1v = vIn1.v
            // Calculate the distance of end points to the line
            val distance0 = Vec2.dot(normal, vIn0v) - offset
            val distance1 = Vec2.dot(normal, vIn1v) - offset
            // If the points are behind the plane
            if (distance0 <= 0.0f) {
                vOut[numOut++].set(vIn0)
            }
            if (distance1 <= 0.0f) {
                vOut[numOut++].set(vIn1)
            }
            // If the points are on different sides of the plane
            if (distance0 * distance1 < 0.0f) {
                // Find intersection point of edge and plane
                val interp = distance0 / (distance0 - distance1)
                val vOutNO = vOut[numOut]
                // vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);
                vOutNO.v.x = vIn0v.x + interp * (vIn1v.x - vIn0v.x)
                vOutNO.v.y = vIn0v.y + interp * (vIn1v.y - vIn0v.y)
                // VertexA is hitting edgeB.
                vOutNO.id.indexA = vertexIndexA.toByte()
                vOutNO.id.indexB = vIn0.id.indexB
                vOutNO.id.typeA = ContactID.Type.VERTEX.ordinal.toByte()
                vOutNO.id.typeB = ContactID.Type.FACE.ordinal.toByte()
                ++numOut
            }
            return numOut
        }

        // djm pooling
        private val d = Vec2()
    }

    init {
        incidentEdge[0] = ClipVertex()
        incidentEdge[1] = ClipVertex()
        clipPoints1[0] = ClipVertex()
        clipPoints1[1] = ClipVertex()
        clipPoints2[0] = ClipVertex()
        clipPoints2[1] = ClipVertex()
    }
}
