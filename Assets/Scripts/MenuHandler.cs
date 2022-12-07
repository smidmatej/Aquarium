using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
public class MenuHandler : MonoBehaviour
{
    public static bool GameIsPaused = false;

    //public GameObject PauseMenu;
    private GameObject PauseMenu;
    private GameObject SettingsMenu;
    private GameObject StartMenu;

    void Awake() {
        PauseMenu = transform.Find("PauseMenu").gameObject;
        SettingsMenu = transform.Find("SettingsMenu").gameObject;
        StartMenu = transform.Find("StartMenu").gameObject;
    }

    // Update is called once per frame
    void Update()
    {
        if (StartMenu.activeSelf == false)
        {
            // StartMenu ignores all keyboard inputs
            if (Input.GetKeyDown(KeyCode.Escape))
            {
                if (PauseMenu.activeSelf == true && SettingsMenu.activeSelf == false)
                {   
                    // If the pause menu is open, close it
                    Resume();
                }
                else
                {
                    // Game is running or the settings menu is open
                    OpenPauseMenu();
                }
            }
        }
    }

    public void Resume()
    {
        PauseMenu.SetActive(false);
        SettingsMenu.SetActive(false);
        Time.timeScale = 1f;
        GameIsPaused = false;
    }

    public void OpenPauseMenu()
    {
        PauseMenu.SetActive(true);
        SettingsMenu.SetActive(false);
        Time.timeScale = 0f;
        GameIsPaused = true;
    }

    public void OpenSettingsMenu()
    {
        PauseMenu.SetActive(false);
        SettingsMenu.SetActive(true);
        Time.timeScale = 0f;
        GameIsPaused = true;
    }

    public void LoadGame()
    {
        Time.timeScale = 1f;
        SceneManager.LoadScene("Game");
    }


    public void Quit()
    {
        Debug.Log("Quitting game...");
        Application.Quit();
    }

    public void LoadMenu()
    {
        Time.timeScale = 1f;
        SceneManager.LoadScene("StartMenu");
    }
}
