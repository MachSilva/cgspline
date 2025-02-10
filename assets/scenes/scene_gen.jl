using Printf
using StaticArrays

# Bézier point gen
ys = [0, 1, 0, -1, 0, 1, 0, 1]
for i in CartesianIndices((0:6,0:6))
  a,b = i[1]/6, i[2]/6
  y = 0.5 * (ys[1+i[1]] + ys[2+i[2]])
  @printf("%2.6f,%2.6f,%2.6f\n", 1 - 2a, y, 1 - 2b)
end

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

begin
  sun = spherical2xz_y(400, deg2rad(30), deg2rad(70))
  preamble =
  """
  let f0 = (0.04 0.04 0.04)
  let Gold = (0.8 0.498039 0.196078)
  let Copper = (0.72 0.45 0.20)
  let Bronze = (0.55 0.47 0.14)
  let Silver = (0.90 0.91 0.98)
  let DarkGray = (0.025 0.025 0.025)
  let SkyBlue = (0.2422812 0.61720675 0.8307701) // From_sRGB(135 206 235)
  let Views =
  {
    GroundCloseB: { position:(-2 1.1 -11) rotation:(-14 -160 -0.00091) }
    GroundClose: { position:(3.5 2.8 14) rotation:(-15 16 -0.00096) }
    Up: { position:(3 2.9 3.5) rotation:(-34 20 -0.0012) }
    HighUp: { position:(10 13 13) rotation:(-40 36 -0.0012) }
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

  let io = open("assets/scenes/test02.dl", "w+")
    print(io, preamble)

    for i in 1:1
      print(io, "{name:\"Light $i\" light:{} transform:{position:(0 10 0)}}\n")
    end

    print(io, afterlights)

    print(io, "{name:\"teapot\" transform:{rotation:(-90 0 0)} surface:{model:Get(\"teapot\") material:Get(\"Gold\")}}")

    c = [5, 8, 8, 8] # count
    r = [3, 5.5, 8, 9] # radii
    θ = [0, 0, 17, -4.5]
    for i in eachindex(r)
      print(io, "{ name:\"Set $i\" transform:{ rotation:(0 $(θ[i]) 0) } objects:[\n")
      for j in 1:c[i]
        φ = (2π / c[i]) * (j-1)
        a = rad2deg(φ)
        z, x = r[i] .* sincos(φ)

        name = "teacup"
        m = materials[mod1(47j + 13i, length(materials))]
        print(io, "{ name:\"$name $i-$j\"",
          " transform:{ position:$(p(x,0,z)) rotation:$(p(0,a,0)) }",
          " surface:{ model:Get(\"$name\") material:Get(\"$m\") } }\n")
      end
      print(io, "]}\n")
    end

    print(io, prologue)
    close(io)
  end

  let io = open("assets/scenes/test03.dl", "w+")
    print(io, preamble)

    r = collect(3:3:12) # radii

    nlights = 0
    for i in eachindex(r)
      c = 2i + 1
      for j in 1:c
        φ = (2π / c) * (j-1) + 0.2j
        z, x = r[i] .* sincos(φ)
    
        nlights += 1
        print(io, "{name:\"Light $nlights (S$i R$j)\" light:{color:*(0.7 (1 1 1))} transform:{position:$(p(x,4,z))}}\n")
      end
    end

    print(io, afterlights)

    print(io, "{name:\"teapot\" transform:{rotation:(-90 0 0)} surface:{model:Get(\"teapot\") material:Get(\"Silver\")}}\n")

    c = [5, 8, 8, 8] # count
    r = [3, 5.5, 8, 9] # radii
    θ = [0, 0, 17, -4.5]
    scales = Vec3[(0.9,0.9,0.9), (0.8,0.8,0.8)]
    for i in eachindex(r)
      print(io, "{ name:\"Set $i\" transform:{ rotation:(0 $(θ[i]) 0) } objects:[\n")
      for j in 1:c[i]
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
      end
      print(io, "]}\n")
    end

    print(io, prologue)
    close(io)
  end
end
