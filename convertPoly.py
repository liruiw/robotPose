import os
import sys
import vtk
import collada

def read_dae(inFile):
    #print inFile
    f = collada.Collada(inFile, ignore=[collada.DaeUnsupportedError])
    polyDataList = colladaSceneToPolyData(f.scene)
    actorList = []
    for i, polyData in enumerate(polyDataList):
        mat = polyData[1]
        poly_data = polyData[0]
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(poly_data)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        for prop, value in mat:#shining is ignored
           # print prop
            # if prop == 'diffuse':
            #     actor.GetProperty().SetDiffuse(value[0])
            # if prop == 'ambient':
            #     actor.GetProperty().SetAmbient(value[0])
            # if prop == 'specular':
            #     actor.GetProperty().SetSpecular(value[0])
            if prop == 'diffuse':
                actor.GetProperty().SetDiffuseColor(value[:3])
            if prop == 'ambient':
                actor.GetProperty().SetAmbientColor(value[:3])
            if prop == 'specular':
                actor.GetProperty().SetSpecularColor(value[:3])
        #print "returned by me"
        actorList.append(actor)
        #print actorList
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
        #uv = tcoords[ti]

        points.SetPoint(i, xyz)
        #tcoordArray.SetTuple2(i, uv[0], uv[1])

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
   # polyData.GetPointData().SetTCoords(tcoordArray)
    return (polyData,materials)


def initMaterialsLibrary(materials):

    global materialsLibrary
    materialsLibrary = {}

    for material in materials:
        materialsLibrary[material.name] = material


def initMaterialsGeometry(geom):

    global materialsGeometry
    materialsGeometry = {}

    for materialNode in geom.materials:
        materialsGeometry[materialNode.symbol] = materialNode.target


def addTextureMetaData(polyData, materialName):

    global materialsGeometry
    global materialsLibrary
    material = materialsGeometry.get(materialName) or materialsLibrary.get(materialName)

    if not material:
        print 'texture: none'
        return

    textureFile = material.effect.diffuse.sampler.surface.image.path
    print 'texture:', textureFile

    if not os.path.isfile(textureFile):
        print 'warning, cannot find texture file:', textureFile

    s = vtk.vtkStringArray()
    s.InsertNextValue(textureFile)
    s.SetName('texture_filename')
    polyData.GetFieldData().AddArray(s)


def colladaGeometryToPolyData(scene):
    polyDataList = []
    for geom in scene.objects('geometry'):
        #print geom.primitives()
        for prim in geom.primitives():
            if type(prim) == collada.polylist.BoundPolylist:
                prim = prim.triangleset() # won't work
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
            #addTextureMetaData(polyData, primitives.material)
           # polyData = transformPolyData(polyData)
            polyDataList.append(polyData)

    return polyDataList


def colladaPolygonToPolyData(tris, material):
    nfaces = len(tris.vertex_index)
    vertex_index = tris.vertex_index
    normal_index = tris.normal_index
    #texcoord_index = tris.texcoord_indexset[0]
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
            #else:
            #    print 'found duplicate vertex and texcoord pair'

            face.append(vtId)

        faces.append(face)

    return createPolyData(faces, vtList, tris.vertex, material)


def colladaTransformToVtk(node):

    assert node.matrix.shape == (4,4)

    m = vtk.vtkMatrix4x4()

    for r in xrange(4):
        for c in xrange(4):
            m.SetElement(r, c, node.matrix[r][c])
    t = vtk.vtkTransform()
    t.SetMatrix(m)
    return t


def colladaNodeToPolyData(node, transform):

    if type(node) == collada.scene.GeometryNode:
        initMaterialsGeometry(node)
        return colladaGeometryToPolyData(node, transform)#give collada

    elif type(node) == collada.scene.Node:

        t = colladaTransformToVtk(node)
        t.PostMultiply()
        t.Concatenate(transform)

        return colladaNodesToPolyData(node.children, t)

    else:
        return []

def colladaTransformToVtk(node):

    assert node.matrix.shape == (4,4)

    m = vtk.vtkMatrix4x4()

    for r in xrange(4):
        for c in xrange(4):
            m.SetElement(r, c, node.matrix[r][c])

    t = vtk.vtkTransform()
    t.SetMatrix(m)
    return t

def colladaNodesToPolyData(nodes, transform):
    return [polyData for node in nodes for polyData in colladaNodeToPolyData(node, transform)]


def colladaSceneToPolyData(scene):#would there be transformation
    #return colladaNodesToPolyData(scene.nodes, vtk.vtkTransform())
    return colladaGeometryToPolyData(scene)

def colladaToPolyData(inFile, outFile):

    #print 'reading:', inFile
    f = collada.Collada(inFile, ignore=[collada.DaeUnsupportedError])

    # initMaterialsLibrary(f.materials)
    polyDataList = colladaSceneToPolyData(f.scene)
    #polyDataList = colladaToPolyData(f)
    if not polyDataList:
        return

if __name__ == '__main__':

    if len(sys.argv) < 2:
        print 'Usage: %s <collada files>' % sys.argv[0]
        sys.exit(1)

    for inFile in sys.argv[1:]:
        outFile = os.path.splitext(inFile)[0] + '.vtm'
        colladaToPolyData(inFile, outFile)


