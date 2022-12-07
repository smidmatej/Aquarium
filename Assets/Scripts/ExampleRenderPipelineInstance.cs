using UnityEngine;
using UnityEngine.Rendering;
    
public class ExampleRenderPipelineInstance : RenderPipeline
{
    public ExampleRenderPipelineInstance() {
    }
    
    protected override void Render (ScriptableRenderContext context, Camera[] cameras) {
        // This is where you can write custom rendering code. Customize this method to customize your SRP.
    }
}