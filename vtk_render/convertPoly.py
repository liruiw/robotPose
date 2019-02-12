import os
import sys
import vtk
import collada

def read_dae(inFile):
    f = collada.Collada(inFile, ignore=[collada.DaeUnsupportedError])
    polyDataList = colladaSceneToPolyData(f.scene)
    actorList = []
    for polyData in polyDataList:
        mat = polyData[1]
        poly_data = polyData[0]
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(poly_data)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        for prop, value in mat:#shining is ignored
            if prop == 'diffuse':
                actor.GetProperty().SetDiffuseColor(value[:3])
            if prop == 'ambient':
                actor.GetProperty().SetAmbientColor(value[:3])
            if prop == 'specular':
                actor.GetProperty().SetSpecularColor(value[:3])
        actorList.append(actor)
    return actorList


def createPolyData(faces, vtList, verts, materials):

    points = vtk.vtkPoints()
    points.SetDataTypeToDouble()
    points.SetNumberOfPoints(len(vtList))

    tcoordArray = vtk.vtkDoubleArray()
    tcoordArray.SetName('tcoords')
    tcoordArray.SetNumberOfComponents(2)
    tcoordArray.SetNumberOfTuples(len(vtList))

    for i, vt in enumerate(vtList):
        vi = vt
        xyz = verts[vi]

        points.SetPoint(i, xyz)

    cells = vtk.vtkCellArray()
    for i, face in enumerate(faces):
        tri = vtk.vtkTriangle()
        tri.GetPointIds().SetId(0, face[0])
        tri.GetPointIds().SetId(1, face[1])
        tri.GetPointIds().SetId(2, face[2])
        cells.InsertNextCell(tri)

    polyData = vtk.vtkPolyData()
    polyData.SetPoints(points)
    polyData.SetPolys(cells)
    return (polyData,materials)


def colladaGeometryToPolyData(scene):
    polyDataList = []
    for geom in scene.objects('geometry'):
        for prim in geom.primitives():
            if type(prim) == collada.polylist.BoundPolylist:
                prim = prim.triangleset() 
            mat = prim.material
            material = []
            polyData = None
            if mat:
                for prop in mat.effect.supported:
                    value = getattr(mat.effect, prop)
                    if value is not None:
                        material.append((prop, value))
                polyData = colladaPolygonToPolyData(prim, material)
            if not polyData:
                continue
            polyDataList.append(polyData)

    return polyDataList


def colladaPolygonToPolyData(tris, material):
    nfaces = len(tris.vertex_index)
    vertex_index = tris.vertex_index
    normal_index = tris.normal_index
    vtList = []
    vtMap = {}
    faces = []

    for triVertInds, triNormalInds in zip(vertex_index, normal_index):
        face = []
        for i in xrange(3):
            vi, ni = triVertInds[i], triNormalInds[i]
            vt = (vi)

            vtId = vtMap.get(vt)
            if not vtId:
                vtList.append(vt)
                vtId = len(vtList)-1
                vtMap[vt] = vtId
            face.append(vtId)
        faces.append(face)
    return createPolyData(faces, vtList, tris.vertex, material)

def colladaSceneToPolyData(scene):
    return colladaGeometryToPolyData(scene)

def colladaToPolyData(inFile, outFile):
    f = collada.Collada(inFile, ignore=[collada.DaeUnsupportedError])
    polyDataList = colladaSceneToPolyData(f.scene)
    if not polyDataList:
        return

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print 'Usage: %s <collada files>' % sys.argv[0]
        sys.exit(1)

    for inFile in sys.argv[1:]:
        outFile = os.path.splitext(inFile)[0] + '.vtm'
        colladaToPolyData(inFile, outFile)


