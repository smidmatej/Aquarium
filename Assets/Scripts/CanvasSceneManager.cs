using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
public class CanvasSceneManager : MonoBehaviour
{

    // Start is called before the first frame update
    void Start()
    {
        //Debug.Log(SceneManager.GetSceneByName("StartMenu").IsValid());
        if(SceneManager.GetSceneByName("StartMenu").IsValid() == false) {
            transform.Find("StartMenu").gameObject.SetActive(false);
        }

        
        if(SceneManager.GetSceneByName("StartMenu").IsValid() == true) {
            transform.Find("StartMenu").gameObject.SetActive(true);
        }
        
    }

}
