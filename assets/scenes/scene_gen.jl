using Printf
using StaticArrays

# Bézier point gen
#= ys = [0, 1, 0, -1, 0, 1, 0, 1]
for i in CartesianIndices((0:6,0:6))
  a,b = i[1]/6, i[2]/6
  y = 0.5 * (ys[1+i[1]] + ys[2+i[2]])
  @printf("%2.6f,%2.6f,%2.6f\n", 1 - 2a, y, 1 - 2b)
end =#

#= let
  # xs = [-1, -0.4, 0.4, 1]
  # ys = [0, 1, 2, 3]
  # zs = [0, 1, 1, 0]
  xs = collect(range(-1, 1, length=7))
  ys = [0, 1, 2, 3]
  zs = repeat([0, 1], 4)
  for I in CartesianIndices((4:7,1:4))
    i, j = Tuple(I)
    x, y, z = xs[i], ys[j], zs[i]
    println("$x,$y,$z")
  end
end =#

const Vec2 = SVector{2,Float64}
const Vec3 = SVector{3,Float64}
const Vec4 = SVector{4,Float64}

# Spherical coordinates to XYZ coordinates with Z-axis containing the poles
function spherical2xy_z(r, θ, φ)
  w, z = sincos(θ)
  y, x = sincos(φ)
  Vec3(r*w*x, r*w*y, r*z)
end

# Spherical coordinates to XYZ coordinates with Y-axis containing the poles
function spherical2xz_y(r, θ, φ)
  x, y, z = spherical2xy_z(r, θ, φ)
  Vec3(x, z, y)
end

fr(e) = round(e, digits=4)
p(x,y,z) = "($(fr(x)) $(fr(y)) $(fr(z)))"
p(v::Vec3) = p(v...)

function gen_test(nsets)
  sun = spherical2xz_y(400, deg2rad(30), deg2rad(70))
  preamble =
  """
  let f0 = (0.04 0.04 0.04)
  let Bronze      = (0.8  0.498039    0.196078)
  let Copper      = (0.72 0.45 0.20)
  let Gold        = (0.55 0.47 0.14)
  let Silver = (0.90 0.91 0.98)
  let DarkGray = (0.025 0.025 0.025)
  let SkyBlue = (0.2422812 0.61720675 0.8307701) // From_sRGB(135 206 235)
  let Views =
  {
    GroundClose: { position:(3.5 2.8 14) rotation:(-15 16 -0.00096) }
    Up: { position:(3 2.9 3.5) rotation:(-34 20 -0.0012) }
    HighUp: { position:(10 13 13) rotation:(-40 36 -0.0012) }
    CameraB: { position:(4.2 1.9 -1.6) rotation:(-20 120 -0.001) }
  }
  Material { name:"Silver" metalness:0.99 roughness: 0.01 diffuse:f0 specular:Silver }
  Material { name:"Gold" metalness:0.99 roughness: 0.01 diffuse:f0 specular:Gold }
  Material { name:"Bronze" metalness:0.99 roughness: 0.01 diffuse:f0 specular:Bronze }
  Material { name:"Red" metalness:0.1 roughness: 0.9 diffuse:(0.98 0.04 0.04) specular:(0.98 0.04 0.04) }
  Material { name:"Blueish" metalness:0.3 roughness: 0.9 diffuse:(0.04 0.6 0.9) specular:(0.04 0.6 0.9) }
  Material { name:"CeramicWhite" metalness:0.4 roughness:0.11 diffuse:(0.82 0.82 0.82) specular:(0.82 0.82 0.82) }
  Surface { name:"teacup" file:"../bezier/teacup" }
  Surface { name:"teapot" file:"../bezier/teapot" }
  Surface { name:"teaspoon" file:"../bezier/teaspoon" }
  Surface { name:"longwave" file:"../bezier/longwave" }
  Scene
  {
    environment:{ background:(0.18 0.18 0.2) ambient:DarkGray }
    view:Views.HighUp
    objects:
    [
      { name:"Floor" surface:{model:Get("longwave") material:Get("CeramicWhite")} transform:{position:(0 -0.1 0) scale:(20 0.1 20)} }
      {
        name:"Lights"
        objects:
        [
          {name:\"Sun\" light:{color:*(1.2e5 (1 0.95 0.8))} transform:{position:$(p(sun))}}
  """
  afterlights =
  """
        ]
      }
  """
  prologue =
  """
    ]
  }
  """
  materials = ["Gold","Silver","Bronze","Red","Blueish"]

  initialobjs = 1 + 1 # floor and central teapot
  # nsets = 34
  objcount = initialobjs + 3nsets
  let io = open("assets/scenes/Test$objcount.dl", "w+")
    print(io, preamble)

    nlights = nsets
    layer = 0
    i = 0
    while i < nlights
      layer += 1
      c = 3layer
      for j in 1:c
        i ≥ nlights && break

        φ = (2π / c) * (j-1) + 0.2j
        z, x = c .* sincos(φ)
    
        i += 1
        print(io, "{name:\"Light $i (S$layer R$j)\" light:{color:*(0.7 (1 1 1))} transform:{position:$(p(x,4,z))}}\n")
      end
    end
    println("light count = $(i+1)") # plus sun

    print(io, afterlights)

    print(io, "{ name:\"teapot\" transform:{rotation:(-90 0 0)} surface:{model:Get(\"teapot\") material:Get(\"Silver\")} }\n")

    c = [5, 8, 8, 10, 12, 14, 17] # layer object count
    r = [3, 5.5, 8, 10, 12, 14] # layers radii
    θ = [0, 0, 17, -4.5, -17, 34, -34] # layer start offsets
    scales = Vec3[(0.9,0.9,0.9), (0.8,0.8,0.8)]
    n = 0
    i = 1 # layer
    while n < nsets
      print(io, "{ name:\"Set $i\" transform:{ rotation:(0 $(θ[i]) 0) } objects:[\n")
      for j in 1:c[i]
        # println("n=$n, i=$i, j=$j")
        n ≥ nsets && break
        φ = (2π / c[i]) * (j-1)
        a = rad2deg(φ)
        z, x = r[i] .* sincos(φ)

        k = 29j + 13i # pseudo random-like number
        m = materials[mod1(k, length(materials))]
        s = scales[mod1(k, length(scales))]
        print(io,
        """
        { name:"$i-$j"
          transform:{ position:$(p(x,0,z)) rotation:$(p(0,a,0)) scale:$(p(s)) }
          objects:[
          { name:"teacup" transform:{} surface:{ model:Get("teacup") material:Get("$m") } }
          { name:"teaspoon-l" transform:{ position:(0 0.09 1.6) rotation:(-85 40 60) } surface:{ model:Get("teaspoon") material:Get("$m") } }
          { name:"teaspoon-r" transform:{ position:(1.2 0.09 -0.5) rotation:(-85 40 180) } surface:{ model:Get("teaspoon") material:Get("$m") } }
        ] }\n""")
        n += 1
      end
      print(io, "]}\n")
      i += 1
    end
    
    print(io, prologue)
    close(io)
    println("n sets = $n, objcount = $objcount")
  end
end

gen_test.(10:8:54)
