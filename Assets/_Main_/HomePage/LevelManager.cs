using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class LevelManager : MonoBehaviour
{
    [Serializable]
    public class LevelData
    {
        [Header("Level Content")]
        public Sprite icon;

        public string title;

        [TextArea(2, 6)]
        public string body;

        [Header("Scene")]
        public string sceneName;

        [Header("Lock Settings")]
        public bool isLocked;
    }

    [Header("Level Data")]
    [SerializeField]
    private List<LevelData> levels = new List<LevelData>();

    [Header("Prefab")]
    [SerializeField]
    private LevelBlockItem levelBlockPrefab;

    [Header("Parent")]
    [Tooltip("Content object inside your Scroll View.")]
    [SerializeField]
    private Transform itemsParent;

    [Header("Animation")]
    [SerializeField]
    private bool animateOnStart = true;

    [Tooltip("Delay between each level item appearing.")]
    [SerializeField]
    private float delayBetweenItems = 0.12f;

    [Tooltip("Duration for the complete item to fade in.")]
    [SerializeField]
    private float itemFadeDuration = 0.2f;

    [Tooltip("Duration for Icon, Title and Body fades.")]
    [SerializeField]
    private float elementFadeDuration = 0.15f;

    [Tooltip("Delay between Icon -> Title -> Body.")]
    [SerializeField]
    private float delayBetweenElements = 0.04f;

    private readonly List<LevelBlockItem> spawnedItems =
        new List<LevelBlockItem>();

    private Coroutine animationCoroutine;

    private void Start()
    {
        PopulateLevels();
    }

    public void PopulateLevels()
    {
        if (levelBlockPrefab == null)
        {
            Debug.LogError("LevelBlockPrefab has not been assigned.", this);
            return;
        }

        if (itemsParent == null)
        {
            Debug.LogError("Items Parent has not been assigned.", this);
            return;
        }

        ClearLevels();

        foreach (LevelData levelData in levels)
        {
            LevelBlockItem newItem =
                Instantiate(levelBlockPrefab, itemsParent);

            newItem.Setup(levelData);
            newItem.HideImmediate();

            spawnedItems.Add(newItem);
        }

        Canvas.ForceUpdateCanvases();

        RectTransform parentRect = itemsParent as RectTransform;

        if (parentRect != null)
            LayoutRebuilder.ForceRebuildLayoutImmediate(parentRect);

        if (animateOnStart)
        {
            animationCoroutine = StartCoroutine(AnimateItems());
        }
        else
        {
            ShowAllImmediately();
        }
    }

    private IEnumerator AnimateItems()
    {
        foreach (LevelBlockItem item in spawnedItems)
        {
            if (item == null)
                continue;

            StartCoroutine(
                item.AnimateIn(
                    itemFadeDuration,
                    elementFadeDuration,
                    delayBetweenElements
                )
            );

            if (delayBetweenItems > 0)
                yield return new WaitForSecondsRealtime(delayBetweenItems);
        }

        animationCoroutine = null;
    }

    public void ShowAllImmediately()
    {
        foreach (LevelBlockItem item in spawnedItems)
        {
            if (item == null)
                continue;

            if (item.itemCg != null)
                item.itemCg.alpha = 1f;

            if (item.iconCg != null)
                item.iconCg.alpha = 1f;

            if (item.titleCg != null)
                item.titleCg.alpha = 1f;

            if (item.bodyCg != null)
                item.bodyCg.alpha = 1f;

            item.RefreshInteractionState();
        }
    }

    public void ClearLevels()
    {
        if (animationCoroutine != null)
        {
            StopCoroutine(animationCoroutine);
            animationCoroutine = null;
        }

        foreach (LevelBlockItem item in spawnedItems)
        {
            if (item != null)
                Destroy(item.gameObject);
        }

        spawnedItems.Clear();
    }
}