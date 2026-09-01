using System.Collections;
using TMPro;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class LevelBlockItem : MonoBehaviour
{
    [Header("UI References")]
    public Image iconImg;
    public TMP_Text title;
    public TMP_Text body;

    [Header("Canvas Groups")]
    public CanvasGroup iconCg;
    public CanvasGroup titleCg;
    public CanvasGroup bodyCg;
    public CanvasGroup itemCg;

    [Header("Button")]
    public Button itemBtn;

    private string sceneName;
    private bool isLocked;

    public void Setup(LevelManager.LevelData data)
    {
        if (data == null)
            return;

        sceneName = data.sceneName;
        isLocked = data.isLocked;

        if (iconImg != null)
        {
            iconImg.sprite = data.icon;
            iconImg.enabled = data.icon != null;
        }

        if (title != null)
            title.text = data.title;

        if (body != null)
            body.text = data.body;

        SetupCanvasGroups();

        if (itemBtn != null)
        {
            itemBtn.onClick.RemoveAllListeners();
            itemBtn.onClick.AddListener(OpenLevel);

            // Locked level = button cannot be clicked.
            itemBtn.interactable = !isLocked;
        }
    }

    private void SetupCanvasGroups()
    {
        SetupChildCanvasGroup(iconCg);
        SetupChildCanvasGroup(titleCg);
        SetupChildCanvasGroup(bodyCg);

        RefreshInteractionState();
    }

    private void SetupChildCanvasGroup(CanvasGroup cg)
    {
        if (cg == null)
            return;

        cg.interactable = false;
        cg.blocksRaycasts = false;
    }

    public void HideImmediate()
    {
        SetAlpha(itemCg, 0f);
        SetAlpha(iconCg, 0f);
        SetAlpha(titleCg, 0f);
        SetAlpha(bodyCg, 0f);

        if (itemCg != null)
        {
            itemCg.interactable = false;
            itemCg.blocksRaycasts = false;
        }
    }

    public IEnumerator AnimateIn(
        float itemDuration,
        float elementDuration,
        float delayBetweenElements)
    {
        if (itemCg != null)
        {
            itemCg.interactable = false;
            itemCg.blocksRaycasts = false;
        }

        yield return FadeCanvasGroup(
            itemCg,
            0f,
            1f,
            itemDuration
        );

        yield return FadeCanvasGroup(
            iconCg,
            0f,
            1f,
            elementDuration
        );

        if (delayBetweenElements > 0)
            yield return new WaitForSecondsRealtime(delayBetweenElements);

        yield return FadeCanvasGroup(
            titleCg,
            0f,
            1f,
            elementDuration
        );

        if (delayBetweenElements > 0)
            yield return new WaitForSecondsRealtime(delayBetweenElements);

        yield return FadeCanvasGroup(
            bodyCg,
            0f,
            1f,
            elementDuration
        );

        RefreshInteractionState();
    }

    public void RefreshInteractionState()
    {
        // Child CanvasGroups should NEVER block raycasts.
        SetupChildCanvasGroup(iconCg);
        SetupChildCanvasGroup(titleCg);
        SetupChildCanvasGroup(bodyCg);

        if (itemCg != null)
        {
            itemCg.alpha = 1f;

            /*
             * itemCg can still receive raycasts because the Button
             * itself handles whether the item is clickable.
             */
            itemCg.interactable = true;
            itemCg.blocksRaycasts = true;
        }

        if (itemBtn != null)
        {
            itemBtn.interactable = !isLocked;
        }
    }

    public void SetLocked(bool locked)
    {
        isLocked = locked;

        RefreshInteractionState();
    }

    public bool IsLocked()
    {
        return isLocked;
    }

    private IEnumerator FadeCanvasGroup(
        CanvasGroup cg,
        float from,
        float to,
        float duration)
    {
        if (cg == null)
            yield break;

        cg.alpha = from;

        if (duration <= 0)
        {
            cg.alpha = to;
            yield break;
        }

        float timer = 0f;

        while (timer < duration)
        {
            timer += Time.unscaledDeltaTime;

            float t = Mathf.Clamp01(timer / duration);

            // Cubic Ease Out
            t = 1f - Mathf.Pow(1f - t, 3f);

            cg.alpha = Mathf.Lerp(from, to, t);

            yield return null;
        }

        cg.alpha = to;
    }

    private void SetAlpha(CanvasGroup cg, float alpha)
    {
        if (cg != null)
            cg.alpha = alpha;
    }

    private void OpenLevel()
    {
        // Extra protection in case this gets called manually.
        if (isLocked)
            return;

        if (string.IsNullOrWhiteSpace(sceneName))
        {
            Debug.LogWarning(
                $"No scene has been assigned to level: {title?.text}",
                this
            );

            return;
        }

        SceneManager.LoadScene(sceneName);
    }
}